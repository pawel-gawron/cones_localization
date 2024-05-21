#include "cones_localization/kalman_filter.hpp"

KF::KF(float initial_x,
        float initial_y,
        float accel_variance) {

    _x[iX] = initial_x;
    _x[iV] = initial_y;

    _accel_variance = accel_variance;

    _P.setIdentity();

}

void KF::reinitial(float reinitial_x,
                float reinitial_y){
    _x[iX] = reinitial_x;
    _x[iV] = reinitial_y;

    _P.setIdentity();
}

void KF::predict(float dt) {
    Eigen::Matrix<float, NUMVARS, NUMVARS> F;
    F.setIdentity();
    F.coeffRef(iX, iV) = dt;

    std::cout << std::endl;

    Eigen::Matrix<float, NUMVARS, 1> new_x;

    new_x = F * _x;

    Eigen::Matrix<float, NUMVARS, 1> G;

    G[iX] = 0.5 * dt * dt;
    G[iV] = dt;

    Eigen::Matrix<float, NUMVARS, NUMVARS> new_P;

    new_P = F * _P * F.transpose() + G * G.transpose() * _accel_variance;

    _P = new_P;
    _x = new_x;
}

void KF::update(float meas_value, float meas_variance)
{
    Eigen::Matrix<float, 1, NUMVARS> H;
    H.setZero();
    H(0, iX) = 1;

    Eigen::Matrix<float, 1, 1> z;
    Eigen::Matrix<float, 1, 1> R;

    z(0) = meas_value;
    R(0) = meas_variance;

    Eigen::Matrix<float, 1, 1> y = z - H * _x;
    Eigen::Matrix<float, 1, 1> S = H * _P * H.transpose() + R;

    Eigen::Matrix<float, NUMVARS, 1> K = _P * H.transpose() * S.inverse();

    Eigen::Matrix<float, NUMVARS, 1> new_x = _x + K * y;

    Eigen::Matrix<float, NUMVARS, NUMVARS> eye = Eigen::Matrix<float, NUMVARS, NUMVARS>::Identity();

    Eigen::Matrix<float, NUMVARS, NUMVARS> new_P = (eye - K * H) * _P;

    _P = new_P;
    _x = new_x;
}

Eigen::Matrix<float, 2, 1> KF::get_mean() {
    return _x;
}

Eigen::Matrix<float, 2, 2> KF::get_cov() {
    return _P;
}