#ifndef PUBLISH_H
#define PUBLISH_H
#include <cartbot/utility.h>

/* Clustering Node Publish Function */
static void setBox(const Object &cluster, jsk_recognition_msgs::BoundingBox &box)
{
    box.header.frame_id = "laser_frame";
    box.header.stamp = ros::Time::now();
    box.label = cluster.id - 1;
    box.dimensions.x = cluster.width + 0.1;
    box.dimensions.y = cluster.height + 0.1;
    box.dimensions.z = 0.3;
    box.pose.position.x = cluster.x;
    box.pose.position.y = cluster.y;
    box.pose.position.z = 0;
    box.value = 1;
}

static void setTextMarker(const Object &cluster, visualization_msgs::Marker &marker)
{
    marker.header.frame_id = "laser_frame";
    marker.header.stamp = ros::Time::now();
    marker.text = "x: " + std::to_string(cluster.x) + " \n" + "y: " + std::to_string(cluster.y) + " \n" + "dist: " + std::to_string(cluster.dist) + " \n" + "id: " + std::to_string(cluster.id);
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.id = cluster.id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = cluster.x;
    marker.pose.position.y = cluster.y;
    marker.pose.position.z = 0.2;
}

/* Tracking Node Publish Function */
static void publishPlot(
    ros::Publisher *x_pub,
    ros::Publisher *y_pub,
    const double mx, // measurement distance
    const double my,
    const double ex, // estimation distance
    const double ey,
    const double mvx, // measurement velocity
    const double mvy,
    const double evx, // estimation velocity
    const double evy)
{
    std_msgs::Float64 x_comp[4], y_comp[4];
    x_comp[0].data = mx;
    y_comp[0].data = my;
    x_comp[1].data = ex;
    y_comp[1].data = ey;
    x_comp[2].data = mvx;
    y_comp[2].data = mvy;
    x_comp[3].data = evx;
    y_comp[3].data = evy;
    for (std::size_t i = 0; i < 4; i++)
    {
        x_pub[i].publish(x_comp[i]);
        y_pub[i].publish(y_comp[i]);
    }
}

static void publishEstimationBox(ros::Publisher &pub, const float &x, const float &y, const int &state) // Observation Boudary Box
{
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = "laser_frame";
    box.header.stamp = ros::Time::now();
    if (state == TRACKING)
        box.label = 4;
    else
        box.label = 2;
    box.dimensions.x = 0.5;
    box.dimensions.y = 0.5;
    box.dimensions.z = 0.01;
    box.pose.position.x = x;
    box.pose.position.y = y;
    box.pose.position.z = 0;
    box.value = 4;
    pub.publish(box);
}

/* Avoiding Node Publish Function */
static void publishTargetLine(ros::Publisher &pub, const double &x1, const double &y1, const double &x2, const double &y2)
{
    visualization_msgs::Marker target_line_marker;
    geometry_msgs::Point p1, p2;
    p1.x = x1;
    p1.y = y1;
    p1.z = 0;
    p2.x = x2;
    p2.y = y2;
    p2.z = 0;
    target_line_marker.points.push_back(p1);
    target_line_marker.points.push_back(p2);
    target_line_marker.header.frame_id = "laser_frame";
    target_line_marker.header.stamp = ros::Time::now();
    target_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    target_line_marker.pose.position.x = 0;
    target_line_marker.pose.position.y = 0;
    target_line_marker.pose.position.z = 0;
    target_line_marker.scale.x = 0.04;
    target_line_marker.scale.y = 0.04;
    target_line_marker.scale.z = 0.04;
    target_line_marker.color.a = 0.9;
    target_line_marker.color.r = 1.0;
    target_line_marker.color.g = 0.0;
    target_line_marker.color.b = 1.0;
    pub.publish(target_line_marker);
}

static void publishCurrnetLine(visualization_msgs::MarkerArray &marker_arr, const double &x1, const double &y1, const double &x2, const double &y2)
{
    visualization_msgs::Marker current_line_marker;
    geometry_msgs::Point p1, p2;
    p1.x = x1;
    p1.y = y1;
    p1.z = 0;
    p2.x = x2;
    p2.y = y2;
    p2.z = 0;
    current_line_marker.points.push_back(p1);
    current_line_marker.points.push_back(p2);
    current_line_marker.header.frame_id = "laser_frame";
    current_line_marker.header.stamp = ros::Time::now();
    current_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    current_line_marker.pose.position.x = 0;
    current_line_marker.pose.position.y = 0;
    current_line_marker.pose.position.z = 0;
    current_line_marker.scale.x = 0.04;
    current_line_marker.scale.y = 0.04;
    current_line_marker.scale.z = 0.04;
    current_line_marker.color.a = 0.9;
    current_line_marker.color.r = 0.0;
    current_line_marker.color.g = 1.0;
    current_line_marker.color.b = 1.0;
    current_line_marker.id = marker_arr.markers.size();
    marker_arr.markers.push_back(current_line_marker);
}

static void publishForceVector(visualization_msgs::MarkerArray &marker_arr, const double &f_x, const double &f_y)
{
    visualization_msgs::Marker force_marker;
    tf2::Quaternion tf_quat;
    const double rad = atan2(f_y, f_x);
    tf_quat.setRPY(0, 0, rad);
    force_marker.header.frame_id = "laser_frame";
    force_marker.header.stamp = ros::Time::now();
    force_marker.type = visualization_msgs::Marker::ARROW;
    force_marker.pose.position.x = 0;
    force_marker.pose.position.y = 0;
    force_marker.pose.position.z = 0;
    force_marker.scale.x = abs(sqrt(pow(f_x, 2) + pow(f_y, 2)));
    force_marker.scale.y = 0.04;
    force_marker.scale.z = 0.04;
    force_marker.color.a = 0.9;
    force_marker.color.r = 0.0;
    force_marker.color.g = 1.0;
    force_marker.color.b = 0.0;
    force_marker.pose.orientation = tf2::toMsg(tf_quat);
    force_marker.id = marker_arr.markers.size();
    marker_arr.markers.push_back(force_marker);
}

static void publishHumanStl(visualization_msgs::MarkerArray &marker_arr, const double &x, const double &y, const double &theta)
{
    visualization_msgs::Marker human_marker;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, theta + DEG2RAD(90.0));
    human_marker.header.frame_id = "laser_frame";
    human_marker.header.stamp = ros::Time::now();
    human_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    human_marker.pose.position.x = x;
    human_marker.pose.position.y = y;
    human_marker.pose.position.z = 0;
    human_marker.scale.x = 0.4;
    human_marker.scale.y = 0.4;
    human_marker.scale.z = 0.4;
    human_marker.color.a = 1.0;
    human_marker.mesh_resource = "package://cartbot/meshes/human.stl";
    human_marker.pose.orientation = tf2::toMsg(tf_quat);
    human_marker.id = marker_arr.markers.size();
    marker_arr.markers.push_back(human_marker);
}

#endif