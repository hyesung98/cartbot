#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include <cartbot/utility.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <ctime>

#define TRACKING_BOUDARY 1
typedef struct
{
    float x;
    float y;
    float weight;
}particle;

class PARTICLE_FILTER
{
    public:
        PARTICLE_FILTER(int num);
        ~PARTICLE_FILTER(){}
        void run();
        void initCoordination(float x, float y);
        void doPrediction(); //motion_model
        void doResampling();
        CLUSTER_STATE doUpdate(const cartbot::clusterarray &clusterlist);
        pcl::PointCloud<pcl::PointXYZ> particle_cloud;
        float getBoundX() {return bound_x;};
        float getBoundY() {return bound_y;};
    private:
        int numofParticles;
        float bound_x, bound_y;
        float weight_sum;
        int lost_cnt;
        std::vector<particle> ptlist;
        bool isNear(cartbot::cluster &cluster);
};

#endif