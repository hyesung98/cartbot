#include <cartbot/utility.h>

void objectListCallback(const cartbot::clusterarray::ConstPtr &objectlist)
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoiding_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber cluster_sub = nh.subscribe<cartbot::clusterarray>("/track/objectlist", 1, objectListCallback);
}