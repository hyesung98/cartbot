#include <cartbot/particle_filter.h>

PARTICLE_FILTER::PARTICLE_FILTER(int num)
{
    numparticles_ = num;
    ptlist_.clear();
    weight_sum_ = 0;
}

void PARTICLE_FILTER::initCoordination(float init_x, float init_y, ros::Time init_time)
{
    ptlist_.clear();
    particle_cloud_.clear();
    bf_time_ = init_time;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(TRACKING_BOUDARY * -100, TRACKING_BOUDARY * 100);
    for (int i = 0; i < numparticles_; i++)
    {
        particle pt;
        pcl::PointXYZ pcl_pt;
        pt.x = init_x + dis(gen) * 0.01;
        pt.y = init_y + dis(gen) * 0.01;
        pt.weight = 0;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = 0;
        ptlist_.push_back(pt);
        particle_cloud_.push_back(pcl_pt);
    }
}

void PARTICLE_FILTER::run(const Model &prediction, const std::vector<Model> &measure)
{
    doPrediction(prediction);
    doUpdate(measure);
    doResampling();
}

void PARTICLE_FILTER::doPrediction(const Model &model)
{
    ros::Time now_time = ros::Time::now();
    ros::Duration delta_t = now_time - bf_time_;
    for (int i = 0; i < numparticles_; i++) // human model
    {
        /* Constant Speed model*/
        ptlist_.at(i).x += model.vx * delta_t.toNSec() + static_cast<float>(getGaussianRandom(0, 0.05));
        ptlist_.at(i).y += model.vy * delta_t.toNSec() + static_cast<float>(getGaussianRandom(0, 0.05));
        /* Acceleration Speed model */
        // ptlist_.at(i).x += model.vx * delta_t.toNSec() + (0.5 * model.ax * pow(delta_t.toNSec(), 2)) + static_cast<float>(getGaussianRandom(0, 0.05));
        // ptlist_.at(i).y += model.vy * delta_t.toNSec() + (0.5 * model.ay * pow(delta_t.toNSec(), 2)) + static_cast<float>(getGaussianRandom(0, 0.05));
        particle_cloud_.at(i).x = ptlist_.at(i).x; // just using Gassian Random Variable
        particle_cloud_.at(i).y = ptlist_.at(i).y;
        ptlist_.at(i).weight = 0;
    }
    bf_time_ = now_time;
}

void PARTICLE_FILTER::doUpdate(const std::vector<Model> &measurelist) // Updating Measure Model
{
    weight_sum_ = 0;
    for(int i = 0 ; i < numparticles_ ; i++)
    {
        float tmp_weight = 0;
        for(auto measure : measurelist)
        {
            tmp_weight += 1 / sqrt(pow(ptlist_.at(i).x - measure.x, 2) + pow(ptlist_.at(i).y - measure.y , 2));
        }
        ptlist_.at(i).weight = tmp_weight;
        weight_sum_ += tmp_weight;
    }
}

void PARTICLE_FILTER::doResampling()
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
    x_ = 0;
    y_ = 0;
    for(auto pt : ptlist_)
    {
        x_ += pt.x;
        y_ += pt.y;
    }
    x_ /= numparticles_;
    y_ /= numparticles_;
}
