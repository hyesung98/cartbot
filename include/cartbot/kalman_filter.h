#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <Eigen/Dense>

class KalmanFilter {

    public :

        KalmanFilter(){};
        void init(const Eigen::MatrixXd& A_, // System Synamics Matrix
                  const Eigen::MatrixXd& C_, // Sensor -> Real Scale Matrix
                  const Eigen::MatrixXd& R_, // Meausurement noise Convariance
                  const Eigen::MatrixXd& P_,
                  const double Q); // Noise Convariance Q Matrix Omega
        void initValue(const Eigen::VectorXd& x0);
        void updateT(double dt);
        void update(const Eigen::VectorXd& z);
        double getX() {return x_hat(0);};
        double getY() {return x_hat(1);};
        double getVX() {return x_hat(2);};
        double getVY() {return x_hat(3);};

    private:
        Eigen::MatrixXd A,C,Q,R,P,K,P0;
        int m,n,c;
        double e;
        bool initialized = false;
        Eigen::MatrixXd I;
        Eigen::VectorXd x_hat;
};
#endif;