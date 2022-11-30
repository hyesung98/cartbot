#include <cartbot/utility.h>
#include <cartbot/publish.h>

ros::Publisher vel_pub, target_pub, marker_arr_pub;
visualization_msgs::MarkerArray marker_arr;

/* PID Variable */
double err_x = 0, err_y = 0;
double target_x, target_y;

/* Potential Field Variable */
double vrep_x = 0, vrep_y = 0;
double vatt_x = 0, vatt_y = 0;
double vpot_x = 0, vpot_y = 0;

/* ROS Parameter Variable*/
double Kp_rel, Kp_att;
double min_dist, u_k;
double weight, wheel_radius, wheel_interval, base_range;
double max_err_x, max_err_y, kp_x, kp_y, ki_x, ki_y;
bool isadvanced;

/* Advanced Accompny System Variable*/
double pre_r, pre_theta;
double pre_pos_l, pre_pos_r, now_pos_l, now_pos_r;
double v_x, v_y;
ros::Time pre_time;

bool getParameter(ros::NodeHandle &nh)
{
    if (!nh.getParam("Kp_rel", Kp_rel))
        return false;
    if (!nh.getParam("Kp_att", Kp_att))
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
    if (!nh.getParam("isadvanced", isadvanced))
        return false;
    if (!nh.getParam("kp_x", kp_x))
        return false;
    if (!nh.getParam("ki_x", ki_x))
        return false;
    if (!nh.getParam("max_err_x", max_err_x))
        return false;
    if (!nh.getParam("kp_y", kp_y))
        return false;
    if (!nh.getParam("ki_y", ki_y))
        return false;
    if (!nh.getParam("max_err_y", max_err_y))
        return false;
    if (!nh.getParam("u_k", u_k))
        return false;

    return true;
}

std::pair<double, double> calcAccompanySystem(const cartbot::Current::ConstPtr &current, double t_x, double t_y)
{
    std::pair<double, double> vel_vector;
    double e_x, e_y, c_x, c_y, phi;

    /* PID Control */
    e_x = t_x - current->x;
    e_y = t_y - current->y;

    if (abs(e_x) < 0.13)
        e_x = 0;

    if (abs(e_y) < 0.13)
        e_y = 0;

    err_x += e_x;
    err_y += e_y;

    if (abs(err_x) > max_err_x)
    {
        if (err_x > 0)
            err_x = max_err_x;
        else
            err_x = -max_err_x;
    }
    if (abs(err_y) > max_err_y)
    {
        if (err_y > 0)
            err_y = max_err_y;
        else
            err_y = -max_err_y;
    }
    c_x = kp_x * e_x + ki_x * err_x;
    c_y = kp_y * e_y + ki_y * err_y;

    vel_vector.second = -(base_range / (base_range + t_x)) * c_y;
    vel_vector.first = -c_x + (t_y / base_range) * vel_vector.second;

    publishHumanStl(marker_arr, current->x, current->y, atan2(current->vy, current->vx));
    publishCurrnetLine(marker_arr, 0, 0, current->x, current->y);

    return vel_vector;
}

std::pair<double, double> calcAdvacnedAccompanySystem(const cartbot::Current::ConstPtr &current, const double &dt, const double &vx, const double &vy, const double &omega, double &t_x, double &t_y)
{
    std::pair<double, double> vel_vector;
    double x0, y0, r, r_dot, theta, theta_dot, phi, err_theta;
    double u_x, u_y, e_x, e_y, c_x, c_y;

    r = sqrt(pow(current->x, 2) + pow(current->y, 2));
    theta = atan2(current->y, current->x);

    /* Calculate User Velocity */
    double err_r = r - pre_r;
    if (abs(err_r) < 0.01)
        err_r = 0;
    r_dot = err_r / dt;

    err_theta = theta - pre_theta;
    if (abs(err_theta) < DEG2RAD(0.05))
        err_theta = 0;
    theta_dot = err_theta / dt;

    u_x = vx + r_dot * cos(theta) - r * (omega + theta_dot) * sin(theta);
    u_y = vy + r_dot * sin(theta) + r * (omega + theta_dot) * cos(theta);

    if (abs(u_x) < 0.00001)
        u_x = 0;
    if (abs(u_y) < 0.00001)
        u_y = 0;

    phi = atan2(u_y, u_x);
    x0 = current->x - r * cos(phi - theta);
    y0 = current->y + r * sin(phi - theta);

    /* PID Controller */
    e_x = t_x - x0;
    e_y = t_y - y0;

    err_x += e_x;
    err_y += e_y;

    if (abs(err_x) > max_err_x)
    {
        if (err_x > 0)
            err_x = max_err_x;
        else
            err_x = -max_err_x;
    }
    if (abs(err_y) > max_err_y)
    {
        if (err_y > 0)
            err_y = max_err_y;
        else
            err_y = -max_err_y;
    }

    if (abs(e_x) < 0.01)
        e_x = 0;

    if (abs(e_y) < 0.01)
        e_y = 0;

    c_x = kp_x * e_x + ki_x * err_x;
    c_y = kp_y * e_y + ki_y * err_y;

    vel_vector.first = c_x * cos(phi) - c_y * sin(phi);
    vel_vector.second = c_x * sin(phi) + c_y * cos(phi);

    publishCurrnetLine(marker_arr, current->x, current->y, x0, y0);
    publishHumanStl(marker_arr, current->x, current->y, phi);

    pre_r = r;
    pre_theta = theta;
    return vel_vector;
}

std::pair<double, double> calcObstacleVelocity(const double vx, std::vector<cartbot::Cluster> objectlist, const double c)
{
    std::pair<double, double> vel_vector(0, 0);
    int cnt = 0;
    for (auto &object : objectlist)
    {
        for (auto &point : object.points)
        {
            if (point.x > 0)
            {
                double r, theta;
                r = sqrt(pow(point.x, 2) + pow(point.y, 2));
                if (r < min_dist)
                {
                    theta = atan2(point.y, point.x);
                    vel_vector.first += vx * -1;
                    vel_vector.second += vx * tan(theta) * -1;
                    cnt++;
                }
            }
        }
    }
    if (abs(vel_vector.first) > 0)
    {
        vel_vector.first = vel_vector.first / cnt * c;
    }
    if (abs(vel_vector.second) > 0)
    {
        vel_vector.second = vel_vector.second / cnt * c;
    }

    return vel_vector;
}

std::pair<double, double> calcRepulsivePotentialField(const double &x, const double &y, std::vector<cartbot::Cluster> objectlist)
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

std::pair<double, double> calcAttractivePotentialField(const double &tx, const double &ty, const double &nx, const double &ny)
{
    std::pair<double, double> force_vector;
    double e_x, e_y, dist;
    e_x = tx - nx;
    e_y = ty - ny;
    dist = sqrt(pow(e_x, 2) + pow(e_y, 2));
    force_vector.first = Kp_att * e_x / dist;
    force_vector.second = Kp_att * e_y / dist;
    return force_vector;
}

void currentCallback(const cartbot::Current::ConstPtr &current)
{
    std::pair<double, double> vel_vector, rep_force, att_force;
    geometry_msgs::Twist cmd_vel;
    uint8_t state = current->state;
    double w_r, w_l;
    marker_arr.markers.clear();
    if (state == cartbot::Current::LOST)
    {
        err_x = 0;
        err_y = 0;
        v_x = 0;
        v_y = 0;
    }
    if (state == cartbot::Current::INIT)
    {
        if (isadvanced == true)
        {
            /* Advanced Accompany System */
            double r, theta, phi = 0;
            r = sqrt(pow(current->x, 2) + pow(current->y, 2));
            theta = atan2(current->y, current->x);
            target_x = current->x + -r * cos(phi - theta);
            target_y = current->y + r * sin(phi - theta);

            pre_r = r;
            pre_theta = theta;
            pre_time = ros::Time::now();
            publishTargetLine(target_pub, current->x, current->y, target_x, target_y);
        }
        else
        {
            /* Accompany System */
            target_x = current->x;
            target_y = current->y;
            publishTargetLine(target_pub, 0, 0, current->x, current->y);
        }
    }
    if (state == cartbot::Current::TRACKING)
    {
        /* Time Calculation */
        ros::Time now_time = ros::Time::now();
        ros::Duration duration_t = now_time - pre_time;
        const double dt = duration_t.toSec();

        /* Velocity Calculation */
        double omega, vx, vy, err_pos_r, err_pos_l;

        err_pos_r = now_pos_r - pre_pos_r;
        if (abs(err_pos_r) < DEG2RAD(0.05))
            err_pos_r = 0;
        err_pos_l = now_pos_l - pre_pos_l;
        if (abs(err_pos_l) < DEG2RAD(0.05))
            err_pos_l = 0;

        w_r = err_pos_r / dt;
        w_l = err_pos_l / dt;

        omega = (wheel_radius / wheel_interval) * (w_r - w_l);
        vx = (wheel_radius / wheel_interval) * wheel_interval / 2 * (w_r + w_l);
        vy = base_range * omega;

        /* Accompany System */
        if (isadvanced == true)
            vel_vector = calcAdvacnedAccompanySystem(current, dt, vx, vy, omega, target_x, target_y);
        else
            vel_vector = calcAccompanySystem(current, target_x, target_y);

        /* Repulsive Potential Field */
        // rep_force = calcRepulsivePotentialField(0, 0, current->objects);
        // att_force = calcAttractivePotentialField(current->x, current->y, target_x, target_y);

        // vrep_x += rep_force.first / weight;
        // vrep_y += rep_force.second / weight;

        // vatt_x += att_force.first / weight;
        // vatt_y += att_force.second / weight;

        // if (abs(vrep_x) > abs(vx))
        // {
        //     if (vrep_x > 0)
        //         vrep_x = abs(vx);
        //     else
        //         vrep_x = -abs(vx);
        // }
        // if (abs(vrep_y) > 1.0)
        // {
        //     if (vrep_y > 0)
        //         vrep_y = 1.0;
        //     else
        //         vrep_y = -1.0;
        // }

        // if (abs(vrep_x) > 0)
        // {
        //     if (vrep_x < 0)
        //     {
        //         vrep_x += u_k * weight;
        //         if (vrep_x > 0)
        //             vrep_x = 0;
        //     }
        //     else
        //     {
        //         vrep_x -= u_k * weight;
        //         if (vrep_x < 0)
        //             vrep_x = 0;
        //     }
        // }
        // if (abs(vrep_y) > 0)
        // {
        //     if (vrep_y < 0)
        //     {
        //         vrep_y += u_k * weight;
        //         if (vrep_y > 0)
        //             vrep_y = 0;
        //     }
        //     else
        //     {
        //         vrep_y -= u_k * weight;
        //         if (vrep_y < 0)
        //             vrep_y = 0;
        //     }
        // }

        // if (abs(vatt_x) > 0)
        // {
        //     if (vrep_x < 0)
        //     {
        //         vatt_x += u_k * weight;
        //         if (vatt_x > 0)
        //             vatt_x = 0;
        //     }
        //     else
        //     {
        //         vatt_x -= u_k * weight;
        //         if (vatt_x < 0)
        //             vatt_x = 0;
        //     }
        // }
        // if (abs(vatt_y) > 0)
        // {
        //     if (vatt_y < 0)
        //     {
        //         vatt_y += u_k * weight;
        //         if (vatt_y > 0)
        //             vatt_y = 0;
        //     }
        //     else
        //     {
        //         vatt_y -= u_k * weight;
        //         if (vatt_y < 0)
        //             vatt_y = 0;
        //     }
        // }

        // vpot_x = vatt_x + vrep_x;
        // vpot_y = vatt_y + vrep_y;
        publishForceVector(marker_arr, rep_force.first, rep_force.second);
        /* Gazebo Sumulation geometry/twist msgs */

        // if (abs(vrep_x) > 0)
        //     v_x = vel_vector.first + vpot_x;
        // else
        v_x = vel_vector.first;
        // if (abs(vrep_y) > 0)
        //     v_y = vel_vector.second + vpot_y;
        // else
        v_y = vel_vector.second;
        // ROS_INFO("vrep_x = %f", vrep_x);
        // ROS_INFO("vrep_y = %f", vrep_y);
        // ROS_INFO("vatt_x = %f", att_force.first);
        // ROS_INFO("vatt_y = %f", att_force.second);
        ROS_INFO("v_x = %f", vel_vector.first);
        ROS_INFO("v_y = %f", vel_vector.second);

        pre_time = now_time;
        pre_pos_r = now_pos_r;
        pre_pos_l = now_pos_l;
    }
    cmd_vel.linear.x = v_x;
    cmd_vel.angular.z = v_y / base_range;
    if (marker_arr.markers.size() > 0)
        marker_arr_pub.publish(marker_arr);
    vel_pub.publish(cmd_vel);
}

void encoderCallback(const sensor_msgs::JointState::ConstPtr &joint)
{
    now_pos_r = joint->position[0];
    now_pos_l = joint->position[1];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoiding_node");
    ros::NodeHandle nh;
    ros::Subscriber target_sub = nh.subscribe<cartbot::Current>("/track/current", 1, currentCallback);
    ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("/robot_1/joint_states", 1, encoderCallback);
    marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("/avoid/marker_array", 1);
    target_pub = nh.advertise<visualization_msgs::Marker>("/avoid/target", 1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel1", 1);
    if (!getParameter(nh))
    {
        ROS_ERROR_STREAM("Check avoiding node parameter");
        exit(0);
    }
    ROS_INFO("\033----> Accompany System & Potential Field Started.\033");
    ros::spin();
}