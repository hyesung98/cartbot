#ifndef DBSCAN_H
#define DBSCAN_H
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cartbot/utility.h>
#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

class DBSCAN
{
public:
    DBSCAN(const unsigned int &minPts, const float &eps, std::vector<Point> &points)
    {
        m_minPoints_ = minPts;
        m_epsilon_ = eps;
        m_points_ = points;
        m_pointSize_ = points.size();
    }
    ~DBSCAN() {}

    int Run();
    std::vector<int> CalculateCluster(Point point);
    int ExpandCluster(Point point, int clusterID);
    inline void CalculateThreshold();
    inline double CalculateDistance(const Point &pointCore, const Point &pointTarget);
    int GetTotalPointSize() { return m_pointSize_; }
    int GetEpsilonSize() { return m_epsilon_; }
    std::vector<Point> GetClusteringData() { return m_points_; };

private:
    std::vector<Point> m_points_;
    std::vector<float> m_thresholds_;
    unsigned int m_pointSize_;
    unsigned int m_minPoints_;
    float m_epsilon_;
};
#endif
