#include <cartbot/utility.h>
#include <cartbot/particle_filter.h>
#include <cartbot/kalman_filter.h>

ros::Publisher estimate_pub;
ros::Publisher x_comp_pub[4];
ros::Publisher y_comp_pub[4];
ros::Publisher object_pub;
ros::Time pre_time;

KalmanFilter kf;
Measurement m;

bool islost = false;
unsigned int lost_cnt = 0;
double estimate_x, estimate_y;
State state = State::LOST;

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

void publishEstimationBox(const float x, const float y) // Observation Boudary Box
{
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = "laser_frame";
    box.header.stamp = ros::Time::now();
    box.label = 10;
    box.dimensions.x = 0.5;
    box.dimensions.y = 0.5;
    box.dimensions.z = 0.01;
    box.pose.position.x = x;
    box.pose.position.y = y;
    box.pose.position.z = 0;
    box.value = 1;
    estimate_pub.publish(box);
}

double evaluateMeasurement(Measurement &m, const double &x, const double &y, const double &dt)
{
    double error_x, error_y;
    double estimate_mx = m.pre_pos[X] + m.pre_vel[X] * dt;
    double estimate_my = m.pre_pos[Y] + m.pre_vel[Y] * dt;
    error_x = abs(x - estimate_mx);
    error_y = abs(y - estimate_my);
    return error_x + error_y;
}

void initKalmanFilter()
{
    int n = 4; // Number of states
    int m = 4; // Number of measurement
    double dt = 0.1;
    const double Q = 10;     // Noise Error Covariance Matrix omega
    Eigen::MatrixXd A(n, n); // System Dynamics Matirx
    Eigen::MatrixXd C(m, n); // Scale Matrix
    Eigen::MatrixXd R(m, m); // Measurement Error Covariance
    Eigen::MatrixXd P(n, n); // Estimate Error Covariance

    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    C << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    R << 0.4, 0, 0.4, 0,
        0, 0.4, 0, 0.4,
        0.4, 0, 0.4, 0,
        0, 0.4, 0, 0.4;

    P << 0.1, 0, 0.1, 0,
        0, 0.1, 0, 0.1,
        0.1, 0, 0.1, 0,
        0, 0.1, 0, 0.1;

    kf.initMatrix(A, C, R, P, Q);
}

void clusterlistCallback(const cartbot::clusterarray::ConstPtr &clusterlist)
{
    constexpr double dist_threshold = 0.5;
    constexpr double dist_min = 0.4;
    cartbot::clusterarray objectlist;
    switch (state)
    {
    case LOST:
    {
        // ROS_INFO("STATE----> LOST");
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x, 2) + pow(cluster.mid_y, 2));
            if (dist < dist_threshold)
            {
                Eigen::VectorXd xn(4);
                estimate_x = cluster.mid_x;
                estimate_y = cluster.mid_y;
                updateMeasurement(m, estimate_x, estimate_y, 0.5);
                pre_time = cluster.Header.stamp;
                xn << estimate_x, estimate_x, 0, 0;
                kf.initValue(xn);
                publishEstimationBox(estimate_x, estimate_y);
                state = TRACKING;
            }
            else
            {
                objectlist.clusters.push_back(cluster);
            }
        }
        break;
    }
    case TRACKING:
    {
        // ROS_INFO("STATE----> TRACKING");
        islost = true;
        std::vector<cartbot::cluster> measure_list;
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x - estimate_x, 2) + pow(cluster.mid_y - estimate_y, 2));
            if (dist < dist_min)
            {
                measure_list.push_back(cluster);
                islost = false;
                lost_cnt = 0;
            }
            else
            {
                objectlist.clusters.push_back(cluster);
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
                    const double dt = duration_t.toSec() + duration_t.toNSec() * 10e-9;
                    error.push_back(evaluateMeasurement(m, measure.mid_x, measure.mid_y, dt));
                }
                double tmp = 100.0;
                for (size_t idx = 0; idx < error.size(); idx++)
                {
                    if (tmp > error.at(idx))
                    {
                        tmp = error.at(idx);
                        m_idx = idx;
                        std::cout << m_idx <<  std::endl;
                    }
                }
            }
            ros::Time now_time = measure_list[m_idx].Header.stamp;
            ros::Duration duration_t = now_time - pre_time;
            const double dt = duration_t.toSec() + duration_t.toNSec() * 10e-9;

            updateMeasurement(m, measure_list[m_idx].mid_x, measure_list[m_idx].mid_y, dt);
            Eigen::VectorXd z(4);
            z << m.now_pos[X], m.now_pos[Y], m.now_vel[X], m.now_vel[Y];
            kf.updateTime(dt);
            kf.processKalmanFilter(z);
            estimate_x = kf.getPosX();
            estimate_y = kf.getPosY();
            publishPlot(m.now_pos[X], m.now_pos[Y], kf.getPosX(), kf.getPosY(),
                        m.now_vel[X], m.now_vel[Y], kf.getVelX(), kf.getVelY());
            publishEstimationBox(estimate_x, estimate_y);
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
    object_pub.publish(objectlist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node");
    ROS_INFO("\033----> Tracking Started.\033");
    ros::NodeHandle nh;
    std::string node_str = "/track";
    std::string topicx_str[4] = {"/measurement_x", "/estimation_x", "/measurement_vx", "/estimation_vx"};
    std::string topicy_str[4] = {"/measurement_y", "/estimation_y", "/measurement_vy", "/estimation_vy"};

    ros::Subscriber clusterlist_sub = nh.subscribe<cartbot::clusterarray>("/cluster/clusterarray", 1, clusterlistCallback);
    estimate_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>(node_str + "/estimation_box", 1);
    object_pub = nh.advertise<cartbot::clusterarray>(node_str + "/objectlist", 1);
    for (std::size_t i = 0; i < 4; i++)
    {
        x_comp_pub[i] = nh.advertise<std_msgs::Float64>(node_str + topicx_str[i], 1);
        y_comp_pub[i] = nh.advertise<std_msgs::Float64>(node_str + topicy_str[i], 1);
    }
    initKalmanFilter();
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}