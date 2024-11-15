#ifndef Z_ESTIMATOR_HPP
#define Z_ESTIMATOR_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class ZEstimator {
private:
    Eigen::MatrixXd B;
    Eigen::MatrixXd G;
    Eigen::MatrixXd I;

public:
    Eigen::MatrixXd K;
    Eigen::MatrixXd F;
    Eigen::MatrixXd f;
    Eigen::MatrixXd P;
    Eigen::MatrixXd u;
    Eigen::MatrixXd z; //Measurement vector
    Eigen::MatrixXd xHat;
    Eigen::MatrixXd H;
    Eigen::MatrixXd h;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd Q0;

    Eigen::MatrixXd P0forFlying;
    Eigen::MatrixXd xHat0;
    Eigen::MatrixXd xHatforFlying;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd R0;
    Eigen::MatrixXd RforFlying;

    //Alpha/beta parameters for partial update
    Eigen::MatrixXd alphaVector;
    Eigen::MatrixXd betaVector; 
    Eigen::MatrixXd gammas;

    ZEstimator();
    void initialize();
    void propagate(double ax, double ay, double az, std::vector<double> q, double dt);
    void update();
    void partialUpdate(double bz);
    void reset();

    Eigen::Matrix3d quat_to_rot(std::vector<double> q);
};

#endif