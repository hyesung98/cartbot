#include <cartbot/particle_filter.h>

Particle_Filter::Particle_Filter(const int num)
{
    numparticles_ = num;
    ptlist_.clear();
    weight_sum_ = 0;
}

void Particle_Filter::InitCoordinate(const float &x, const float &y, const ros::Time &t)
{
    ptlist_.clear();
    particle_cloud_.clear();
    pre_time = t;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(TRACKING_BOUDARY * -100, TRACKING_BOUDARY * 100);
    for (int i = 0; i < numparticles_; i++)
    {
        particle pt;
        pcl::PointXYZ pcl_pt;
        pt.x = x + dis(gen) * 0.01;
        pt.y = y + dis(gen) * 0.01;
        pt.weight = 0;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = 0;
        ptlist_.push_back(pt);
        particle_cloud_.push_back(pcl_pt);
    }
}

void Particle_Filter::Run(const double &vx, const double &vy, const std::vector<measure> &m)
{
    Prediction(vx, vy);
    Update(m);
    Resampling();
}

void Particle_Filter::Prediction(double vx, double vy)
{
    ros::Time now_time = ros::Time::now();
    ros::Duration duration_t = now_time - pre_time;
    double dt = duration_t.toSec() + duration_t.toNSec() * 10e-9;
    for (int i = 0; i < numparticles_; i++)
    {
        /* Constant Speed model*/
        ptlist_.at(i).x += vx * dt + static_cast<float>(getGaussianRandom(0, 0.05));
        ptlist_.at(i).y += vy * dt + static_cast<float>(getGaussianRandom(0, 0.05));
        particle_cloud_.at(i).x = ptlist_.at(i).x;
        particle_cloud_.at(i).y = ptlist_.at(i).y;
        ptlist_.at(i).weight = 0;
    }
    pre_time = now_time;
}

void Particle_Filter::Update(std::vector<measure> measureList)
{
    weight_sum_ = 0;
    for (int i = 0; i < numparticles_; i++)
    {
        float tmp_weight = 0;
        for (auto measure : measureList)
        {
            tmp_weight += 1 / sqrt(pow(ptlist_.at(i).x - measure.x, 2) + pow(ptlist_.at(i).y - measure.y, 2));
        }
        ptlist_.at(i).weight = tmp_weight;
        weight_sum_ += tmp_weight;
    }
}

void Particle_Filter::Resampling()
{
    /* Stochastic Universal Resampling */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, weight_sum_ / numparticles_);
    std::vector<particle> resampled_list;
    float r = dis(gen);
    float c = ptlist_.at(0).weight;
    int i = 0;
    for (int m = 0; m < ptlist_.size(); m++)
    {
        float u = r + m * (weight_sum_ / numparticles_);
        while (u > c)
        {
            i = i + 1;
            c += ptlist_.at(i).weight;
        }
        resampled_list.push_back(ptlist_.at(i));
    }
    ptlist_.clear();
    ptlist_.assign(resampled_list.begin(), resampled_list.end());
    x = 0;
    y = 0;
    for (auto pt : ptlist_)
    {
        x += pt.x;
        y += pt.y;
    }
    x /= numparticles_;
    y /= numparticles_;
}
