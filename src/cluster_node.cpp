#include <cartbot/dbscan.h>
#include <cartbot/utility.h>
ros::Subscriber sub;
ros::Publisher vis_pub;
ros::Publisher pt_pub;
ros::Publisher text_pub;
ros::Publisher cluster_pub;

jsk_recognition_msgs::BoundingBoxArray boxlist;
visualization_msgs::MarkerArray textlist;

pcl::PointCloud<pcl::PointXYZI> cluster_cloud;

float cluster_x, cluster_y;

// DBSCAN parameter
#define MIN_PTS 10
#define MIN_EPS 0.3
#define BOUNDARY 3

void visualizeCluster(const std::vector<Object> &clusterlist)
{
    boxlist.header.frame_id = "laser_frame";
    boxlist.header.stamp = ros::Time::now();

    if (clusterlist.size() > boxlist.boxes.size())
    {
        std::size_t idx = 0;
        std::size_t box_size = boxlist.boxes.size();
        auto it_box = boxlist.boxes.begin();
        auto it_text = textlist.markers.begin();
        for (auto cluster : clusterlist)
        {
            if (idx < box_size)
            {
                setBox(cluster, *it_box);
                setTextMarker(cluster, *it_text);
                it_box++;
                it_text++;
                idx++;
            }
            else
            {
                jsk_recognition_msgs::BoundingBox box;
                visualization_msgs::Marker marker;
                setBox(cluster, box);
                setTextMarker(cluster, marker);
                boxlist.boxes.push_back(box);
                textlist.markers.push_back(marker);
            }
        }
    }
    else if (clusterlist.size() == boxlist.boxes.size())
    {
        auto it = clusterlist.begin();
        for (auto it_box = boxlist.boxes.begin(); it_box != boxlist.boxes.end(); it_box++)
        {
            setBox(*it, *it_box);
            it++;
        }
        it = clusterlist.begin();
        for (auto it_text = textlist.markers.begin(); it_text != textlist.markers.end(); it_text++)
        {
            setTextMarker(*it, *it_text);
            it++;
        }
    }
    else
    {
        std::size_t diff = boxlist.boxes.size() - clusterlist.size();
        boxlist.boxes.erase(boxlist.boxes.end() - diff, boxlist.boxes.end());
        for (auto it_text = textlist.markers.end() - diff; it_text != textlist.markers.end(); it_text++)
            it_text->action = visualization_msgs::Marker::DELETE;
        text_pub.publish(textlist);
        textlist.markers.erase(textlist.markers.end() - diff, textlist.markers.end());
        auto it = clusterlist.begin();
        for (auto it_box = boxlist.boxes.begin(); it_box != boxlist.boxes.end(); it_box++)
        {
            setBox(*it, *it_box);
            it++;
        }
        it = clusterlist.begin();
        for (auto it_text = textlist.markers.begin(); it_text != textlist.markers.end(); it_text++)
        {
            setTextMarker(*it, *it_text);
            it++;
        }
    }
    vis_pub.publish(boxlist);
    text_pub.publish(textlist);
}

void lidar_scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<Point> scan_data;
    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
        if (std::isinf(scan_msg->ranges[i]) == 0 && scan_msg->ranges[i] != 0)
        {
            Point temp_point;
            temp_point.range = scan_msg->ranges[i];
            temp_point.theta = scan_msg->angle_min + i * scan_msg->angle_increment; // RAD unit
            temp_point.x = temp_point.range * COS(temp_point.theta);
            temp_point.y = temp_point.range * SIN(temp_point.theta);
            scan_data.push_back(temp_point);
        }
    }

    /* DBSCAN & ABD Processing*/
    DBSCAN clustering(MIN_PTS, MIN_EPS, scan_data);
    int cluster_size = clustering.run();
    std::vector<Point> clustered_data = clustering.getClusteringData();
    std::vector<Object> object_data(cluster_size);
    cartbot::clusterarray cluster_list;
    cluster_list.clusters.resize(cluster_size);

    /* Classify Point by ID */
    cluster_cloud.clear();
    for (auto pt : clustered_data)
    {
        if (pt.clusterID >= 1)
        {
            pcl::PointXYZI pcl_pt;
            geometry_msgs::Point gm_pt;
            gm_pt.x = pcl_pt.x = pt.x;
            gm_pt.x = pcl_pt.y = pt.y;
            gm_pt.z = pcl_pt.z = 0;
            pcl_pt.intensity = pt.clusterID - 1;
            cluster_cloud.push_back(pcl_pt);
            object_data.at(pt.clusterID - 1).ptlist.push_back(pt);
            cluster_list.clusters.at(pt.clusterID - 1).points.push_back(gm_pt);
        }
    }
    pt_pub.publish(cloud2cloudmsg(cluster_cloud));

    /* Center Point Average Processing */
    int id = 1;
    auto it_cluster = cluster_list.clusters.begin();
    for (auto it_object = object_data.begin(); it_object != object_data.end();)
    {
        double avg_x = 0, avg_y = 0;
        float min_x, max_x, min_y, max_y;
        std::size_t idx = 0;
        for (auto pt : it_object->ptlist)
        {
            if (idx == 0)
            {
                min_x = pt.x;
                max_x = pt.x;
                min_y = pt.y;
                max_y = pt.y;
                idx++;
            }
            if (min_x > pt.x)
                min_x = pt.x;

            if (max_x < pt.x)
                max_x = pt.x;

            if (min_y > pt.y)
                min_y = pt.y;

            if (max_y < pt.y)
                max_y = pt.y;

            avg_x += static_cast<double>(pt.x);
            avg_y += static_cast<double>(pt.y);
        }
        if (avg_x != 0 && avg_y != 0)
        {
            avg_x /= it_object->ptlist.size();
            avg_y /= it_object->ptlist.size();
        }
        else
        {
            avg_x = 0;
            avg_y = 0;
        }

        if (avg_x == 0 && avg_y == 0)
        {
            it_object = object_data.erase(it_object);
            it_cluster = cluster_list.clusters.erase(it_cluster);
        }
        else
        {
            if (abs(avg_x) > BOUNDARY || abs(avg_y) > BOUNDARY)
            {
                it_object = object_data.erase(it_object);
                it_cluster = cluster_list.clusters.erase(it_cluster);
            }
            else
            {
                it_cluster->Header.frame_id = "laser_frame";
                it_cluster->Header.stamp = ros::Time::now();
                it_object->width = abs(max_x - min_x);
                it_object->height = abs(max_y - min_y);
                it_object->dist = sqrt(pow(avg_x, 2) + pow(avg_y, 2));
                it_cluster->mid_x = it_object->x = static_cast<float>(avg_x);
                it_cluster->mid_y = it_object->y = static_cast<float>(avg_y);
                it_cluster->id = it_object->id = id++;
                it_object++;
                it_cluster++;
            }
        }
    }
    cluster_pub.publish(cluster_list);
    visualizeCluster(object_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidar_scan_callback);
    vis_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cluster_boxlist", 1);
    pt_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud", 1);
    text_pub = nh.advertise<visualization_msgs::MarkerArray>("/cluster_text", 1);
    cluster_pub = nh.advertise<cartbot::clusterarray>("cluster_object", 1);
    ROS_INFO("\033----> Clustering Started.\033");
    ros::spin();
    return 0;
}