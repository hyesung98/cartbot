#include <cartbot/utility.h>
#include <cartbot/particle_filter.h>
#include <cartbot/kalman_filter.h>

#define DIST_THRESHOLD 0.5
#define MIN_DIST 0.6
ros::Publisher particle_pub;
ros::Publisher boundary_pub;
ros::Publisher xcomp_pub[4];
ros::Publisher ycomp_pub[4];
std::vector<Model> measure_buffer;
KalmanFilter kf;
int lost_cnt = 0;

CLUSTER_STATE state = CLUSTER_STATE::LOST;

void updatePlot(const double mx, // measurement distance
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
        xcomp_pub[i].publish(x_comp[i]);
        ycomp_pub[i].publish(y_comp[i]);
    }
}

void initKalmanFilter()
{
    int n = 4; // number of states
    int m = 4; // number of measurement
    double dt = 0.1;
    Eigen::MatrixXd A(n, n); // System Dynamics Matirx
    Eigen::MatrixXd C(m, n); // Scale Matrix
    Eigen::MatrixXd R(m, m); // Measurement Error Covariance
    Eigen::MatrixXd P(n, n); // Estimate Error Covariance
    const double Q = 10;    // Noise Error Covariance Matrix omega

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
    kf.init(A, C, R, P, Q);
}

void pusblishBoundary(float bound_x, float bound_y) // Observation Boudary Box
{
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = "laser_frame";
    box.header.stamp = ros::Time::now();
    box.label = 10;
    box.dimensions.x = 0.5;
    box.dimensions.y = 0.5;
    box.dimensions.z = 0.01;
    box.pose.position.x = bound_x;
    box.pose.position.y = bound_y;
    box.pose.position.z = 0;
    box.value = 1;
    boundary_pub.publish(box);
}

/* Particle Filter
void clusterCallback(const cartbot::clusterarray::ConstPtr &clusterlist)
{
    switch (state)
    {
    case LOST:
    {
        ROS_INFO("STATE----> LOST");
        measure_buffer.clear();
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x, 2) + pow(cluster.mid_y, 2));
            if (dist < DIST_THRESHOLD)
            {
                Model tmp_measure;
                tmp_measure.x = cluster.mid_x;
                tmp_measure.y = cluster.mid_y;
                tmp_measure.now_time = cluster.Header.stamp;
                measure_buffer.push_back(tmp_measure);
                pusblishBoundary(tmp_measure.x, tmp_measure.y);
                state = STAGE1;
                break;
            }
        }
        break;
    }

    case STAGE1:
    {
        ROS_INFO("STATE----> STAGE1");
        Model &last_model = measure_buffer[measure_buffer.size() - 1];
        bool islost = false;
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x - last_model.x, 2) + pow(cluster.mid_y - last_model.y, 2));
            if (dist < MIN_DIST)
            {
                Model tmp_measure;
                tmp_measure.x = cluster.mid_x;
                tmp_measure.y = cluster.mid_y;
                tmp_measure.now_time = cluster.Header.stamp;
                ros::Duration delta_t = tmp_measure.now_time - last_model.now_time;
                if (state_cnt >= 0)
                {
                    tmp_measure.vx = (tmp_measure.x - last_model.x) / delta_t.toNSec();
                    tmp_measure.vy = (tmp_measure.y - last_model.y) / delta_t.toNSec();
                    if (state_cnt >= 1)
                    {
                        tmp_measure.ax = (tmp_measure.vx - last_model.vx) / delta_t.toNSec();
                        tmp_measure.ay = (tmp_measure.vy - last_model.vy) / delta_t.toNSec();
                        pf.initCoordination(tmp_measure.x, tmp_measure.y, ros::Time::now());
                        particle_pub.publish(cloud2cloudmsg(pf.getParticle()));
                        state = TRACKING;
                    }
                    measure_buffer.push_back(tmp_measure);
                    pusblishBoundary(tmp_measure.x, tmp_measure.y);
                    islost = true;
                }
                state_cnt++;
                break;
            }
        }
        if (!islost)
        {
            state_cnt = 0;
            state = LOST;
        }
        break;
    }
    case TRACKING:
    {
        ROS_INFO("STATE----> Tracking");
        Model &last_model = measure_buffer[measure_buffer.size() - 1];
        std::vector<Model> particle_model;
        bool islost = false;
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x - last_model.x, 2) + pow(cluster.mid_y - last_model.y, 2));
            if (dist < MIN_DIST)
            {
                Model tmp_model;
                tmp_model.x = cluster.mid_x;
                tmp_model.y = cluster.mid_y;
                particle_model.push_back(tmp_model);
                islost = true;
            }
        }
        if (islost)
        {
            lost_cnt = 0;
            pf.run(measure_buffer.at(measure_buffer.size() - 1), particle_model);
            particle_pub.publish(cloud2cloudmsg(pf.getParticle()));
            pusblishBoundary(pf.getX(), pf.getY());
            Model tmp_model;
            tmp_model.x = pf.getX();
            tmp_model.y = pf.getY();
            tmp_model.now_time = clusterlist->clusters.at(0).Header.stamp;
            ros::Duration delta_t = tmp_model.now_time - last_model.now_time;
            tmp_model.vx = (tmp_model.x - last_model.x) / delta_t.toNSec();
            tmp_model.vy = (tmp_model.y - last_model.y) / delta_t.toNSec();
            tmp_model.ax = (tmp_model.vx - last_model.vx) / delta_t.toNSec();
            tmp_model.ay = (tmp_model.vy - last_model.vy) / delta_t.toNSec();
            measure_buffer.push_back(tmp_model);
            if (measure_buffer.size() > 3)
                measure_buffer.erase(measure_buffer.begin());
        }
        else
        {
            lost_cnt++;
            if (lost_cnt > 5)
            {
                state = LOST;
                lost_cnt = 0;
            }
        }
        break;
    }
    }
}
*/

void clusterCallback(const cartbot::clusterarray::ConstPtr &clusterlist)
{
    switch (state)
    {
    case LOST:
    {
        ROS_INFO("STATE----> LOST");
        measure_buffer.clear();
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x, 2) + pow(cluster.mid_y, 2));
            if (dist < DIST_THRESHOLD)
            {
                Model tmp_measure;
                tmp_measure.x = cluster.mid_x;
                tmp_measure.y = cluster.mid_y;
                tmp_measure.now_time = cluster.Header.stamp;
                Eigen::VectorXd xn(4);
                xn << tmp_measure.x, tmp_measure.y, 0, 0;
                initKalmanFilter();
                kf.initValue(xn);
                pusblishBoundary(tmp_measure.x, tmp_measure.y);
                measure_buffer.push_back(tmp_measure);
                state = TRACKING;
                break;
            }
        }
        break;
    }
    case TRACKING:
    {
        ROS_INFO("STATE----> TRACKING");
        Model &last_model = measure_buffer[measure_buffer.size() - 1];
        bool islost = false;
        for (auto cluster : clusterlist->clusters)
        {
            float dist = sqrt(pow(cluster.mid_x - last_model.x, 2) + pow(cluster.mid_y - last_model.y, 2));
            if (dist < MIN_DIST)
            {
                Model tmp_measure;
                ros::Duration delta_t = cluster.Header.stamp - last_model.now_time;
                double dt = delta_t.toSec() + delta_t.toNSec() * 10e-9;
                tmp_measure.x = cluster.mid_x;
                tmp_measure.y = cluster.mid_y;
                tmp_measure.vx = (tmp_measure.x - last_model.x) / dt;
                tmp_measure.vy = (tmp_measure.y - last_model.y) / dt;
                tmp_measure.now_time = cluster.Header.stamp;
                measure_buffer.push_back(tmp_measure);
                Eigen::VectorXd z(4);
                z << tmp_measure.x, tmp_measure.y, tmp_measure.vx, tmp_measure.vy;
                kf.updateT(dt);
                kf.update(z);
                updatePlot(tmp_measure.x, tmp_measure.y, kf.getX(), kf.getY(),
                           tmp_measure.vx, tmp_measure.vy, kf.getVX(), kf.getVY());
                pusblishBoundary(kf.getX(), kf.getY());
                if (measure_buffer.size() > 2)
                    measure_buffer.erase(measure_buffer.begin());
                break;
            }
        }
        break;
    }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    std::string topicx_str[4] = {"/measurement_x", "/estimation_x", "/measurement_vx", "/estimation_vx"};
    std::string topicy_str[4] = {"/measurement_y", "/estimation_y", "/measurement_vy", "/estimation_vy"};
    ros::Subscriber cluster_sub = nh.subscribe<cartbot::clusterarray>("/cluster_object", 1, clusterCallback);
    particle_pub = nh.advertise<sensor_msgs::PointCloud2>("/particle_cloud", 1);
    boundary_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("/boundary", 1);
    for (std::size_t i = 0; i < 4; i++)
    {
        xcomp_pub[i] = nh.advertise<std_msgs::Float64>(topicx_str[i], 1);
        ycomp_pub[i] = nh.advertise<std_msgs::Float64>(topicy_str[i], 1);
    }
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    ROS_INFO("\033----> Tracking Started.\033");
    return 0;
}