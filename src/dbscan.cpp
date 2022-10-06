#include <cartbot/dbscan.h>

int DBSCAN::run()
{
    int clusterID = 1;
    int index = 0;
    std::vector<Point>::iterator iter;
    calculateThreshold();
    for (iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if (iter->clusterID == UNCLASSIFIED)
        {
            if (expandCluster(*iter, clusterID) != FAILURE)
            {
                clusterID += 1;
            }
        }
    }
    return clusterID;
}

int DBSCAN::expandCluster(Point point, int clusterID)
{
    std::vector<int> clusterSeeds = calculateCluster(point);

    if (clusterSeeds.size() < m_minPoints)
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
            m_points.at(*iterSeeds).clusterID = clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y)
            {
                indexCorePoint = index;
            }
            index++;
        }
        clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

        for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
        {
            std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            if (clusterNeighors.size() >= m_minPoints)
            {
                std::vector<int>::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
                {
                    if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE)
                    {
                        if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED)
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
}

std::vector<int> DBSCAN::calculateCluster(Point point)
{
    int idx = 0;
    std::vector<int> clst_idx;
    std::vector<float>::iterator ths_iter = m_thresholds.begin();
    for (auto pt_iter = m_points.begin(); pt_iter != m_points.end(); ++pt_iter)
    {
        if (calculateDistance(point, *pt_iter) <= *ths_iter)
        {
            clst_idx.push_back(idx);
        }
        idx++;
        ths_iter++;
    }
    return clst_idx;
}

// Adaptive Breakpoint Detector
inline void DBSCAN::calculateThreshold()
{
    float threshold;
    const float l = DEG2RAD(12);
    const float s = 0.13;
    int idx = 0;
    for(auto pt_iter = m_points.begin(); pt_iter !=  m_points.end() ; pt_iter++)
    {
        float dth, dist;
        if(idx == 0)
        {
            dth = pt_iter->theta + m_points.at(m_points.size()-1).theta;
            dist = m_points.at(m_points.size()-1).range;
        }
        else
        {
            dth = pt_iter->theta - (pt_iter-1)->theta;
            dist = (pt_iter - 1)->range;
        }
        threshold = dist * (SIN(dth)) / (SIN(l - dth)) + s * 3;
        m_thresholds.push_back(std::min(threshold, m_epsilon));
        idx++;
    }
}

inline double DBSCAN::calculateDistance(const Point &pointCore, const Point &pointTarget)
{
    return sqrt(pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2));
}
