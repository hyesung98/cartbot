#include <cartbot/utility.h>
#include <cartbot/kalman_filter.h>

ros::Publisher x_comp_pub[4];
ros::Publisher y_comp_pub[4];
ros::Publisher estimate_pub;
ros::Publisher target_pub;
ros::Time pre_time;

/* Kalman Filter Variable */
KalmanFilter kf;
Measurement m;
Estimation e;

bool islost = false;
unsigned int lost_cnt = 0;
State state = State::LOST;

/* ROS Param Variable */
double wait_time;
double dist_threshold;
double min_range;

/* Initialization Variable */
double init_cnt = 0;
double init_x, init_y;
ros::Time init_time;

bool getParameter(ros::NodeHandle &nh)
{
    double sigma_a;
    enum NAME_LIST
    {
        A,
        C,
        R,
        P
    };
    Eigen::MatrixXd mat_A(4, 4);
    Eigen::MatrixXd mat_C(4, 4);
    Eigen::MatrixXd mat_R(4, 4);
    Eigen::MatrixXd mat_P(4, 4);
    XmlRpc::XmlRpcValue paramConfig;

    std::string mat_name[4] = {"A", "C", "R", "P"};
    for (int n = 0; n < 4; n++)
    {
        if (nh.hasParam(mat_name[n]))
        {
            try
            {
                nh.getParam(mat_name[n], paramConfig);
                ROS_ASSERT(paramConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        std::ostringstream ostr;
                        ostr << paramConfig[4 * i + j];
                        std::istringstream istr(ostr.str());
                        switch (n)
                        {
                        case NAME_LIST::A:
                            istr >> mat_A(i, j);
                            break;
                        case NAME_LIST::C:
                            istr >> mat_C(i, j);
                            break;
                        case NAME_LIST::R:
                            istr >> mat_R(i, j);
                            break;
                        case NAME_LIST::P:
                            istr >> mat_P(i, j);
                            break;
                        }
                    }
                }
            }
            catch (...)
            {
                throw;
            }
        }
        else
        {
            return false;
        }
    }
    if (!nh.getParam("sigma_a", sigma_a))
        return false;
    if (!nh.getParam("dist_threshold", dist_threshold))
        return false;
    if (!nh.getParam("min_range", min_range))
        return false;
    if (!nh.getParam("wait_time", wait_time))
        return false;

    std::cout << "A : \n"
              << mat_A << std::endl;

    std::cout << "C : \n"
              << mat_C << std::endl;

    std::cout << "R : \n"
              << mat_R << std::endl;

    std::cout << "P : \n"
              << mat_P << std::endl;

    kf.initMatrix(mat_A, mat_C, mat_R, mat_P, sigma_a);
    return true;
}

void publishPlot(const double mx, // measurement distance
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
        x_comp_pub[i].publish(x_comp[i]);
        y_comp_pub[i].publish(y_comp[i]);
    }
}

void publishEstimationBox(const float &x, const float &y, const int &state) // Observation Boudary Box
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
    estimate_pub.publish(box);
}

double evaluateMeasurement(Estimation e, const double &x, const double &y, const double &dt)
{
    double error_x, error_y;
    double estimate_mx = e.pos[X] + e.vel[Y] * dt;
    double estimate_my = e.pos[Y] + e.vel[Y] * dt;
    error_x = abs(x - estimate_mx);
    error_y = abs(y - estimate_my);
    return error_x + error_y;
}

void clusterlistCallback(const cartbot::ClusterArray::ConstPtr &clusterlist)
{
    cartbot::ClusterArray objectlist;
    cartbot::Target target;
    switch (state)
    {
    case LOST:
    {
        ROS_INFO("STATE----> OBJECT_LOST");
        target.state = state;
        for (auto cluster : clusterlist->Clusters)
        {
            float dist = sqrt(pow(cluster.mid_x, 2) + pow(cluster.mid_y, 2));
            if (dist < dist_threshold)
            {
                init_cnt = 0;
                init_time = ros::Time::now();
                init_x = cluster.mid_x;
                init_y = cluster.mid_y;
                state = COUNT;
                break;
            }
        }
        break;
    }
    case COUNT:
    {
        ROS_INFO("STATE----> TIME_COUNTING");
        target.state = state;
        islost = true;
        for (auto cluster : clusterlist->Clusters)
        {
            float dist = sqrt(pow(init_x - cluster.mid_x, 2) + pow(init_y - cluster.mid_y, 2));
            if (dist < min_range)
            {
                ros::Duration duration_t = ros::Time::now() - init_time;
                init_cnt += duration_t.toSec();
                publishEstimationBox(cluster.mid_x, cluster.mid_y, state);
                islost = false;
                break;
            }
        }
        if (islost)
        {
            state = LOST;
        }
        if (init_cnt > wait_time)
        {
            state = INIT;
        }
        break;
    }
    case INIT:
    {
        ROS_INFO("STATE----> TRACKING_INITIALIZATION");
        target.state = state;
        islost = true;
        for (auto cluster : clusterlist->Clusters)
        {
            float dist = sqrt(pow(init_x - cluster.mid_x, 2) + pow(init_y - cluster.mid_y, 2));
            if (dist < min_range)
            {
                Eigen::VectorXd xn(4);
                updateEstimation(e, cluster.mid_x, cluster.mid_y, 0, 0);
                updateMeasurement(m, e.pos[X], e.pos[Y], 0.5);
                pre_time = cluster.Header.stamp;
                xn << e.pos[Y], e.pos[Y], 0, 0;
                target.x = e.pos[X];
                target.y = e.pos[Y];
                kf.initValue(xn);
                publishEstimationBox(e.pos[X], e.pos[Y], state);
                state = TRACKING;
                islost = false;
            }
            else
            {
                target.objects.push_back(cluster);
            }
        }
        if (islost)
        {
            state = LOST;
        }
        break;
    }
    case TRACKING:
    {
        ROS_INFO("STATE----> TRACKING");
        target.state = state;
        islost = true;
        std::vector<cartbot::Cluster> measure_list;
        for (auto cluster : clusterlist->Clusters)
        {
            float dist = sqrt(pow(cluster.mid_x - e.pos[X], 2) + pow(cluster.mid_y - e.pos[Y], 2));
            if (dist < min_range)
            {
                measure_list.push_back(cluster);
                islost = false;
                lost_cnt = 0;
            }
            else
            {
                target.objects.push_back(cluster);
            }
        }

        if (!islost)
        {
            int m_idx = 0;
            if (measure_list.size() > 1)
            {
                std::vector<double> error;
                for (auto measure : measure_list)
                {
                    ros::Duration duration_t = measure.Header.stamp - pre_time;
                    const double dt = duration_t.toSec();
                    error.push_back(evaluateMeasurement(e, measure.mid_x, measure.mid_y, dt));
                }
                double tmp_err = 100.0;
                for (size_t idx = 0; idx < error.size(); idx++) // find minimum error
                {
                    if (tmp_err > error.at(idx))
                    {
                        tmp_err = error.at(idx);
                        m_idx = idx;
                    }
                }
            }
            ros::Time now_time = measure_list[m_idx].Header.stamp;
            ros::Duration duration_t = now_time - pre_time;
            const double dt = duration_t.toSec();
            updateMeasurement(m, measure_list[m_idx].mid_x, measure_list[m_idx].mid_y, dt);
            Eigen::VectorXd z(4);
            z << m.now_pos[X], m.now_pos[Y], m.now_vel[X], m.now_vel[Y];
            kf.updateTime(dt);
            kf.processKalmanFilter(z);
            updateEstimation(e, kf.getPosX(), kf.getPosY(), kf.getVelX(), kf.getVelY());
            target.Header.stamp = now_time;
            target.x = e.pos[X];
            target.y = e.pos[Y];
            target.vx = e.vel[X];
            target.vy = e.vel[Y];

            publishPlot(m.now_pos[X], m.now_pos[Y], e.pos[X], e.pos[Y],
                        m.now_vel[X], m.now_vel[Y], e.vel[X], e.vel[Y]);
            publishEstimationBox(e.pos[X], e.pos[Y], state);
            pre_time = now_time;
        }
        else
        {
            lost_cnt++;
        }

        if (lost_cnt > 10)
        {
            state = LOST;
        }
        break;
    }
    }
    target_pub.publish(target);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node");
    ROS_INFO("\033----> Tracking Started.\033");
    ros::NodeHandle nh;
    std::string node_str = "/track";
    std::string topicx_str[4] = {"/measurement_x", "/estimation_x", "/measurement_vx", "/estimation_vx"};
    std::string topicy_str[4] = {"/measurement_y", "/estimation_y", "/measurement_vy", "/estimation_vy"};

    ros::Subscriber clusterlist_sub = nh.subscribe<cartbot::ClusterArray>("/cluster/clusterarray", 1, clusterlistCallback);
    estimate_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>(node_str + "/estimation_box", 1);
    target_pub = nh.advertise<cartbot::Target>(node_str + "/target", 1);

    for (std::size_t i = 0; i < 4; i++)
    {
        x_comp_pub[i] = nh.advertise<std_msgs::Float64>(node_str + topicx_str[i], 1);
        y_comp_pub[i] = nh.advertise<std_msgs::Float64>(node_str + topicy_str[i], 1);
    }
    if (!getParameter(nh))
    {
        ROS_ERROR_STREAM("Check tracking node parameter!");
        exit(0);
    }
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}