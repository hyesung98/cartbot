#ifndef UTILITY_H
#define UTILITY_H
#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>
#include <random>
#include <math.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <cartbot/cluster.h>
#include <cartbot/clusterarray.h>
// Math function define
#define RAD2DEG(rad) rad *(180 / M_PI)
#define DEG2RAD(deg) deg * 174533e-07
#define COS cos
#define SIN sin

enum CLUSTER_STATE
{
    LOST,
    STAGE1,
    STAGE2,
    STAGE3,
    TRACKING,
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
    float x, y;
    float vx, vy;
    float ax, ay;
    ros::Time now_time;
} Model;

typedef struct
{
    float x, y;
    std::vector<Point> ptlist;
    int id;
    float width;
    float height;
    float dist;
} Object;

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
    marker.header.frame_id = "laser_frame"; // map frame 기준
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