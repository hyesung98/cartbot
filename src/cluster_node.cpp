#include "ros/ros.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(2);
    while (ros::ok())
    {

    }
    return 0;
}