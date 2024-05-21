#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

class KF {
    private:
        static const int iX = 0;
        static const int iV = 1;
        static const int NUMVARS = iV + 1;
        Eigen::Matrix<float, NUMVARS, 1> _x;
        Eigen::Matrix<float, NUMVARS, NUMVARS> _P;

        float _accel_variance;

    public:           
        KF(float initial_x,
            float initial_y,
            float accel_variance);

        void reinitial(float reinitial_x,
                        float reinitial_y);
        void predict(float dt);
        void update(float meas_value, float meas_variance);
        Eigen::Matrix<float, NUMVARS, 1> get_mean();
        Eigen::Matrix<float, NUMVARS, NUMVARS> get_cov();
};


#endif