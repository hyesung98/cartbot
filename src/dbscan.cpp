#include <cartbot/dbscan.h>

int DBSCAN::Run()
{
    int clusterID = 1;
    int index = 0;
    std::vector<Point>::iterator iter;
    CalculateThreshold();
    for (iter = m_points_.begin(); iter != m_points_.end(); ++iter)
    {
        if (iter->clusterID == UNCLASSIFIED)
        {
            if (ExpandCluster(*iter, clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }
    return clusterID;
}

int DBSCAN::ExpandCluster(Point point, int clusterID)
{
    std::vector<int> clusterSeeds = CalculateCluster(point);

    if (clusterSeeds.size() < m_minPoints_)
    {
        point.clusterID = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points_.at(*iterSeeds).clusterID = clusterID;
            if (m_points_.at(*iterSeeds).x == point.x && m_points_.at(*iterSeeds).y == point.y)
            {
                indexCorePoint = index;
            }
            index++;
        }
        clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

        for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            std::vector<int> clusterNeighors = CalculateCluster(m_points_.at(clusterSeeds[i]));

            if (clusterNeighors.size() >= m_minPoints_)
            {
                std::vector<int>::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
                {
                    if (m_points_.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points_.at(*iterNeighors).clusterID == NOISE)
                    {
                        if (m_points_.at(*iterNeighors).clusterID == UNCLASSIFIED)
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points_.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
}

std::vector<int> DBSCAN::CalculateCluster(Point point)
{
    int idx = 0;
    std::vector<int> clst_idx;
    std::vector<float>::iterator ths_iter = m_thresholds_.begin();
    for (auto pt_iter = m_points_.begin(); pt_iter != m_points_.end(); ++pt_iter)
    {
        if (CalculateDistance(point, *pt_iter) <= *ths_iter)
        {
            clst_idx.push_back(idx);
        }
        idx++;
        ths_iter++;
    }
    return clst_idx;
}

// Range Query
inline void DBSCAN::CalculateThreshold()
{
    float threshold;
    const float l = DEG2RAD(12);
    const float s = 0.13;
    int idx = 0;
    for(auto pt_iter = m_points_.begin(); pt_iter !=  m_points_.end() ; pt_iter++)
    {
        float dth, dist;
        if(idx == 0)
        {
            dth = pt_iter->theta + m_points_.at(m_points_.size()-1).theta;
            dist = m_points_.at(m_points_.size()-1).range;
        }
        else
        {
            dth = pt_iter->theta - (pt_iter-1)->theta;
            dist = (pt_iter - 1)->range;
        }
        // Adaptive Breakpoint Detector
        threshold = dist * (SIN(dth)) / (SIN(l - dth)) + s * 3;
        m_thresholds_.push_back(std::min(threshold, m_epsilon_));
        idx++;
    }
}

inline double DBSCAN::CalculateDistance(const Point &pointCore, const Point &pointTarget)
{
    return sqrt(pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2));
}
