#include <cartbot/utility.h>
#include <cartbot/particle_filter.h>

#define DIST_THRESHOLD 0.4
ros::Publisher particle_pub;
ros::Publisher boundary_pub;

CLUSTER_STATE state = CLUSTER_STATE::LOST;
PARTICLE_FILTER pf(3000);

void pusblishBoundary(float bound_x, float bound_y) // Observation Boudary Box
{
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = "laser_frame";
    box.header.stamp = ros::Time::now();
    box.label = 10;
    box.dimensions.x = 1;
    box.dimensions.y = 1;
    box.dimensions.z = 0.01;
    box.pose.position.x = bound_x;
    box.pose.position.y = bound_y;
    box.pose.position.z = 0;
    box.value = 1;
    boundary_pub.publish(box);
}

void clusterCallback(const cartbot::clusterarray::ConstPtr &clusterlist)
{
    switch (state)
    {
    case LOST:
        for (auto cluster : clusterlist->clusters)
        {
            float distance = sqrt(pow(cluster.mid_x, 2) + pow(cluster.mid_y, 2));
            if (distance < DIST_THRESHOLD)
            {
                pf.initCoordination(cluster.mid_x, cluster.mid_y);
                particle_pub.publish(cloud2cloudmsg(pf.particle_cloud));
                pusblishBoundary(pf.getBoundX(), pf.getBoundY());
                state = STAGE1;
            }
        }
        break;
    case STAGE1:
        state = TRACKING;
        break;
    case STAGE2:
        break;
    case STAGE3:
        break;
    case TRACKING:
        pf.doPrediction();
        pf.doUpdate(*clusterlist);
        pusblishBoundary(pf.getBoundX(), pf.getBoundY());
        particle_pub.publish(cloud2cloudmsg(pf.particle_cloud));
        pf.doResampling();
        particle_pub.publish(cloud2cloudmsg(pf.particle_cloud));
        break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ros::Subscriber cluster_sub = nh.subscribe<cartbot::clusterarray>("/cluster_object", 1, clusterCallback);
    particle_pub = nh.advertise<sensor_msgs::PointCloud2>("/particle_cloud", 1);
    boundary_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("/boundary", 1);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    ROS_INFO("\033----> Tracking Started.\033");
    return 0;
}