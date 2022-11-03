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

typedef struct
{
    double x, y;
}measure;

class Particle_Filter
{
    public:
        Particle_Filter(const int num);
        ~Particle_Filter(){}
        void InitCoordinate(const float &x, const float &y, const ros::Time &t);
        void Run(const double &vx, const double &vy, const std::vector<measure> &m);
        pcl::PointCloud<pcl::PointXYZ> & getParticle() {return particle_cloud_;};
        const float GetX() { return x; };
        const float GetY() { return y; };
    private:
        ros::Time pre_time;
        int numparticles_;
        float weight_sum_;
        float x, y;
        std::vector<particle> ptlist_;
        pcl::PointCloud<pcl::PointXYZ> particle_cloud_;
        void Prediction(double vx, double vy);
        void Update(std::vector<measure> measureList);
        void Resampling();
};

#endif