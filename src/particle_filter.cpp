#include <cartbot/particle_filter.h>

PARTICLE_FILTER::PARTICLE_FILTER(int num)
{
    numofParticles = num;
    ptlist.clear();
}

void PARTICLE_FILTER::initCoordination(float x, float y)
{
    bound_x = x;
    bound_y = y;
    lost_cnt = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(TRACKING_BOUDARY * -100, TRACKING_BOUDARY * 100);
    for (int i = 0; i < numofParticles; i++)
    {
        particle pt;
        pcl::PointXYZ pcl_pt;
        pt.x = bound_x + dis(gen) * 0.01;
        pt.y = bound_y + dis(gen) * 0.01;
        pt.weight = 0;
        pt.weight = 0;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = 0;
        ptlist.push_back(pt);
        particle_cloud.push_back(pcl_pt);
    }
}

void PARTICLE_FILTER::doPrediction()
{
    for (int i = 0; i < numofParticles; i++) // human model
    {
        particle_cloud.at(i).x = ptlist.at(i).x = ptlist.at(i).x + static_cast<float>(getGaussianRandom(0, 0.1)); // just using Gassian Random Variable
        particle_cloud.at(i).y = ptlist.at(i).y = ptlist.at(i).y + static_cast<float>(getGaussianRandom(0, 0.1));
        ptlist.at(i).weight = 0;
    }
}

CLUSTER_STATE PARTICLE_FILTER::doUpdate(const cartbot::clusterarray &clusterlist) // Updating Measure Model
{
    if(lost_cnt > 50)
        return CLUSTER_STATE::LOST;

    // Get clusters which are located near boudary
    std::vector<Point> random_ptlist;
    std::vector<Point> target_ptlist;
    float m_avg, m_sum = 0;
    for (auto cluster : clusterlist.clusters)
    {
        if (cluster.points.size() > 100) // Too many points
            continue;
        if (isNear(cluster))
        {
            Point target_pt;
            target_pt.x = cluster.mid_x;
            target_pt.y = cluster.mid_y;
            target_ptlist.push_back(target_pt);
            for (auto point : cluster.points)
            {
                Point random_pt;
                random_pt.x = point.x;
                random_pt.y = point.y;
                random_ptlist.push_back(random_pt);
            }
        }
    }
    /* Make Measurement Model */
    if (target_ptlist.size() > 0)
    {
        float avg_target_x = 0, avg_target_y = 0;
        for (auto pt : target_ptlist)
        {
            avg_target_x += pt.x;
            avg_target_y += pt.y;
        }
        bound_x = avg_target_x = avg_target_x / target_ptlist.size();
        bound_y = avg_target_y = avg_target_y / target_ptlist.size();
        if (random_ptlist.size() > 12)
            random_ptlist.erase(random_ptlist.begin() + 6, random_ptlist.end() - 6);

        /* mdist --> Target, pdist --> Proposal */
        for (auto pt : random_ptlist)
            m_sum += sqrt(pow(pt.x - avg_target_x, 2) + pow(pt.y - avg_target_y, 2));
        m_avg = m_sum / random_ptlist.size();

        /* Calculate weight of particle */
        weight_sum = 0;
        for (auto ptc : ptlist)
        {
            float ratio, p_avg, p_sum = 0;
            for (auto pt : random_ptlist)
                p_sum += sqrt(pow(ptc.x - avg_target_x, 2) + pow(ptc.y - avg_target_y, 2));
            p_avg = p_sum / random_ptlist.size();
            ratio = p_avg / m_avg;
            if (ratio <= 1.2 && ratio >= 0.8)
            {
                ptc.weight = 1 - (abs(p_sum - m_sum) / m_sum);
            }
            else
            {
                ptc.weight = 0;
            }
            weight_sum += ptc.weight;
        }
        return CLUSTER_STATE::TRACKING;
    }
    else
    {
        lost_cnt++;
        return CLUSTER_STATE::TRACKING;
    }
}

void PARTICLE_FILTER::doResampling()
{
    /* Stochastic Universal Resampling */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, weight_sum / numofParticles);
    std::vector<particle> resampled_list;
    float r = dis(gen);
    float c = ptlist.at(0).weight;
    int i = 0;
    for(int m = 0 ; m < numofParticles ; m++)
    {
        float u = r + (m - 1) * (weight_sum / numofParticles);
        while(u > c)
        {
            i = i + 1;
            std::cout << i << std::endl;
            c = c + ptlist.at(i).weight;
        }
        resampled_list.push_back(ptlist.at(i));
    }
    ptlist.clear();
    ptlist = resampled_list;
}

bool PARTICLE_FILTER::isNear(cartbot::cluster &cluster)
{
    if (cluster.mid_x <= bound_x + TRACKING_BOUDARY && cluster.mid_x >= bound_x - TRACKING_BOUDARY)
    {
        if (cluster.mid_y <= bound_y + TRACKING_BOUDARY && cluster.mid_y >= bound_y - TRACKING_BOUDARY)
        {
            return true;
        }
    }
    return false;
}
