#ifndef UTILITY_H
#define UTILITY_H
#include <ros/ros.h>
#include <vector>
#include <thread>
#include <map>
#include <iostream>
#include <random>
#include <math.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cartbot/Cluster.h>
#include <cartbot/ClusterArray.h>
#include <cartbot/Target.h>
// Math function define
#define RAD2DEG(rad) rad *(180 / M_PI)
#define DEG2RAD(deg) deg * 174533e-07
#define COS cos
#define SIN sin

enum State
{
    LOST,
    COUNT,
    INIT,
    TRACKING
};

enum Axis
{
    X,
    Y
};

typedef struct
{
    float x;
    float y;
    float range;
    float theta;
    bool visited;
    int clusterID = -1;
} Point;

typedef struct
{
    double now_pos[2];
    double now_vel[2];
    double pre_pos[2];
    double pre_vel[2];
} Measurement;

typedef struct
{
    double pos[2];
    double vel[2];
} Estimation;

typedef struct
{
    float x, y;
    std::vector<Point> ptlist;
    int id;
    float width;
    float height;
    float dist;
} Object;

static void updateMeasurement(Measurement &m, const double &x, const double &y, const double &dt)
{
    m.now_pos[X] = x;
    m.now_pos[Y] = y;
    m.now_vel[X] = (m.now_pos[X] - m.pre_pos[X]) / dt;
    m.now_vel[Y] = (m.now_pos[Y] - m.pre_pos[Y]) / dt;
    m.pre_pos[X] = m.now_pos[X];
    m.pre_pos[Y] = m.now_pos[Y];
    m.pre_vel[X] = m.pre_vel[X];
    m.pre_vel[Y] = m.pre_vel[Y];
}

static void updateEstimation(Estimation &e, const double &kf_x, const double &kf_y, const double &kf_vx, const double &kf_vy)
{
    e.pos[X] = kf_x;
    e.pos[Y] = kf_y;
    e.vel[X] = kf_vx;
    e.vel[Y] = kf_vy;
}

static sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZI> &cloud_src)
{
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "laser_frame";
    return cloudmsg;
}

static sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> &cloud_src)
{
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "laser_frame";
    return cloudmsg;
}

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

static double getGaussianRandom(const double m, const double s)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(m, s);
    double number = distribution(gen);
    return number;
}
#endif