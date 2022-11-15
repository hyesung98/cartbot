#include <cartbot/utility.h>

ros::Publisher force_pub, human_pub, control_pub;
visualization_msgs::Marker force_marker, human_marker, control_marker;
double err_x = 0, err_y = 0;
double now_x, now_y;

/* ROS Parameter Variable*/
double Kp_rel;
double min_dist;
double weight, wheel_radius, wheel_interval, base_range;
double max_err, kp, ki;

bool getParameter(ros::NodeHandle &nh)
{
    if (!nh.getParam("Kp_rel", Kp_rel))
        return false;
    if (!nh.getParam("min_dist", min_dist))
        return false;
    if (!nh.getParam("wheel_radius", wheel_radius))
        return false;
    if (!nh.getParam("wheel_interval", wheel_interval))
        return false;
    if (!nh.getParam("weight", weight))
        return false;
    if (!nh.getParam("base_range", base_range))
        return false;
    if (!nh.getParam("kp", kp))
        return false;
    if (!nh.getParam("ki", ki))
        return false;
    if (!nh.getParam("max_err", max_err))
        return false;

    return true;
}

void publishControlPoint(const double &x, const double &y)
{
    control_marker.points.clear();
    geometry_msgs::Point p1, p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    p2.x = x;
    p2.y = y;
    p2.z = 0;
    control_marker.points.push_back(p1);
    control_marker.points.push_back(p2);
    control_marker.header.frame_id = "laser_frame";
    control_marker.header.stamp = ros::Time::now();
    control_marker.type = visualization_msgs::Marker::LINE_STRIP;
    control_marker.pose.position.x = 0;
    control_marker.pose.position.y = 0;
    control_marker.pose.position.z = 0;
    control_marker.scale.x = 0.04;
    control_marker.scale.y = 0.04;
    control_marker.scale.z = 0.04;
    control_marker.color.a = 0.9;
    control_marker.color.r = 1.0;
    control_marker.color.g = 0.0;
    control_marker.color.b = 1.0;
    control_pub.publish(control_marker);
}

void publishForceVector(const double &f_x, const double &f_y)
{
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
    force_pub.publish(force_marker);
}

void publishHumanMarker(const double &x, const double &y, const double &theta)
{
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, theta + DEG2RAD(90));
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
    human_marker.mesh_resource = "package://cartbot/discription/human.stl";
    human_marker.pose.orientation = tf2::toMsg(tf_quat);
    human_pub.publish(human_marker);
}

std::pair<double, double> calcAccompanySystem(cartbot::Target target, double n_x, double n_y)
{
    std::pair<double, double> vel_vector;
    double e_x, e_y, c_x, c_y, theta;
    theta = atan2(target.vy, target.vx);
    e_x = target.x - n_x;
    e_y = target.y - n_y;
    err_x += e_x;
    err_y += e_y;
    if(abs(err_x) > max_err)
    {
        if(err_x > 0)
            err_x = max_err;
        else
            err_x = -max_err;
    }
    if(abs(err_y) > max_err)
    {
        if(err_y > 0)
            err_y = max_err;
        else
            err_y = -max_err;
    }
    c_x = kp * e_x + ki * err_x;
    c_y = kp * e_y + ki * err_y;
    vel_vector.first = c_x * cos(theta) - c_y * sin(theta);
    vel_vector.second = c_y * sin(theta) - c_y * cos(theta);
}

std::pair<double, double> calcPotentialField(const double x, const double y, std::vector<cartbot::Cluster> objectlist)
{
    std::pair<double, double> force_vector(0, 0);
    for (auto &object : objectlist)
    {
        for (auto &point : object.points)
        {
            double dist, dist_x, dist_y;
            dist = abs(sqrt(pow(point.x - x, 2) + pow(point.y - y, 2)));
            dist_x = point.x - x;
            dist_y = point.y - y;
            if (min_dist >= dist)
            {
                force_vector.first -= Kp_rel * (1 / dist - 1 / min_dist) * (1 / pow(dist, 2)) * (dist_x / dist);  // x_repulsive
                force_vector.second -= Kp_rel * (1 / dist - 1 / min_dist) * (1 / pow(dist, 2)) * (dist_y / dist); // y_repulsive
            }
        }
    }
    return force_vector;
}

void targetCallback(const cartbot::Target::ConstPtr &target)
{
    std::pair<double, double> vel_vector, rep_force;
    uint8_t state = target->state;
    double w_r , w_l;
    if(state == cartbot::Target::INIT)
    {
        now_x = target->x;
        now_y = target->y;
        publishControlPoint(now_x, now_y);
        w_l = 0;
        w_r = 0;
    }
    if(state == cartbot::Target::TRACKING)
    {
        publishHumanMarker(target->x, target->y, atan2(target->vy, target->vx));
        rep_force = calcPotentialField(0, 0, target->objects);
        vel_vector = calcAccompanySystem(*target, now_x, now_y);
        std::cout << rep_force.first << std::endl;
        std::cout << rep_force.second << std::endl;
        publishForceVector(rep_force.first, rep_force.second);
        w_l = (1 / wheel_radius) * vel_vector.first - wheel_interval / (wheel_radius * base_range) * vel_vector.second;
        w_r = (1 / wheel_radius) * vel_vector.first + wheel_interval / (wheel_radius * base_range) * vel_vector.second;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoiding_node");
    ros::NodeHandle nh;
    ros::Subscriber target_sub = nh.subscribe<cartbot::Target>("/track/target", 3, targetCallback);
    force_pub = nh.advertise<visualization_msgs::Marker>("/avoid/force", 1);
    human_pub = nh.advertise<visualization_msgs::Marker>("/avoid/human", 1);
    control_pub = nh.advertise<visualization_msgs::Marker>("/avoid/control", 1);
    if (!getParameter(nh))
    {
        ROS_ERROR_STREAM("Check avoiding node parameter");
        exit(0);
    }
    ROS_INFO("\033----> Accompany System & Potential Field Started.\033");
    ros::spin();
}