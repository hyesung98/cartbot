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


class DBSCAN {
public:
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> &points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run();
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline void calculateThreshold();
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    std::vector<Point> getClusteringData(){return m_points;};
public:
    std::vector<Point> m_points;
    std::vector<float> m_thresholds;

private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};
#endif
