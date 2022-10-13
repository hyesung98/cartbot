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
        void initCoordination(const float, const float, ros::Time);
        void run(const Model &, const std::vector<Model> &);
        pcl::PointCloud<pcl::PointXYZ> & getParticle() {return particle_cloud_;};
        float getX() { return x_; };
        float getY() { return y_; };
    private:
        ros::Time bf_time_;
        int numparticles_;
        float x_, y_;
        float weight_sum_;
        std::vector<particle> ptlist_;
        pcl::PointCloud<pcl::PointXYZ> particle_cloud_;
        void doPrediction(const Model &);
        void doUpdate(const std::vector<Model> &);
        void doResampling();
};

#endif