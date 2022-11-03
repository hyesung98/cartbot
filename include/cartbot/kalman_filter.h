#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <Eigen/Dense>

class KalmanFilter
{

public:
    KalmanFilter(){};
    void initMatrix(const Eigen::MatrixXd &A_, // System Synamics Matrix
                    const Eigen::MatrixXd &C_, // Sensor -> Real Scale Matrix
                    const Eigen::MatrixXd &R_, // Meausurement noise Convariance
                    const Eigen::MatrixXd &P_,
                    const double &Q); // Noise Convariance Q Matrix Omega
    void initValue(const Eigen::VectorXd &x0);
    void updateTime(const double &dt);
    void processKalmanFilter(const Eigen::VectorXd &z);
    const double getPosX() { return x_hat(0); };
    const double getPosY() { return x_hat(1); };
    const double getVelX() { return x_hat(2); };
    const double getVelY() { return x_hat(3); };

private:
    Eigen::MatrixXd A, C, Q, R, P, K, P0;
    int m, n, c;
    double e;
    bool initialized = false;
    Eigen::MatrixXd I;
    Eigen::VectorXd x_hat;
};
#endif;