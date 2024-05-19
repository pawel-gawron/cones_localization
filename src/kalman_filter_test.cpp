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
            float accel_variance) {

            _x[iX] = initial_x;
            _x[iV] = initial_y;

            _accel_variance = accel_variance;

            _P.setIdentity();

        }

        void predict(float dt) {
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

        void update(float meas_value, float meas_variance)
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

        Eigen::Matrix<float, NUMVARS, 1> get_mean() {
            return _x;
        }

        Eigen::Matrix<float, NUMVARS, NUMVARS> get_cov() {
            return _P;
        }

};