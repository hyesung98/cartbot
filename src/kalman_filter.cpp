#include <iostream>
#include <vector>
#include <cartbot/kalman_filter.h>

void KalmanFilter::initMatrix(const Eigen::MatrixXd &A_,
                        const Eigen::MatrixXd &C_,
                        const Eigen::MatrixXd &R_,
                        const Eigen::MatrixXd &P_,
                        const double &Q)
{
    e = Q;
    A = A_;
    C = C_;
    R = R_;
    P0 = P_;
    m = C.rows();
    n = A.rows();
    x_hat.setZero();
    x_hat.resize(n);
    I.resize(n, n);
    I.setIdentity();
    initialized = true;
}

void KalmanFilter::updateTime(const double &dt)
{
    if (!initialized)
    {
        std::cout << "Filter is not initialized" << std::endl;
    }
    Eigen::Matrix2d tmp_A;
    tmp_A << dt, 0,
             0, dt;
    A.block<2, 2>(0,2) = tmp_A;

    Q.resize(n, n);
    Eigen::Array22d tmp_Q1, tmp_Q2, tmp_Q3;
    tmp_Q1 << pow(dt, 4) * e / 4, 0,
              0, pow(dt, 4) * e / 4;
    tmp_Q2 << pow(dt, 3) * e / 2, 0,
              0, pow(dt, 3) * e / 2;
    tmp_Q3 << pow(dt, 2) * e, 0,
              0, pow(dt, 2) * e;
    Q.block<2,2>(0,0) = tmp_Q1;
    Q.block<2,2>(0,2) = tmp_Q2;
    Q.block<2,2>(2,0) = tmp_Q2;
    Q.block<2,2>(2,2) = tmp_Q3;
}

void KalmanFilter::initValue(const Eigen::VectorXd& x0)
{
    x_hat = x0;
    P = P0;
    initialized = true;
}

void KalmanFilter::processKalmanFilter(const Eigen::VectorXd &z) // Sensor input matrix
{
    if (!initialized)
    {
        std::cout << "Filter is not initialized" << std::endl;
    }

    /* Prediction */
    x_hat = A * x_hat;             // State Space Equation
    P = A * P * A.transpose() + Q; // Calculate Covariance Matrix

    /* Measurement */
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (z - C * x_hat);
    P = (I - K * C) * P;
}