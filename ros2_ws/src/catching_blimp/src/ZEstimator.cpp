#include "ZEstimator.hpp"

#include <iostream>

ZEstimator::ZEstimator() {

    //Initial state
    F = Eigen::MatrixXd(3,3);
    B = Eigen::MatrixXd(3,1);

    G = Eigen::MatrixXd(3,2);
    G << 0, 0,
         1, 0,
         0, 1;

    K = Eigen::MatrixXd::Zero(3,1);
    u = Eigen::MatrixXd::Zero(1,1);
    z = Eigen::MatrixXd::Zero(1,1);

    I = Eigen::MatrixXd::Identity(3,3);

    H = Eigen::MatrixXd(1,3);
    H << 1, 0, 0;

    xHat = Eigen::MatrixXd(3,1);

    //P0 is created to save the initial covariance values. It keeps its value forever.
    P0 = Eigen::MatrixXd(3,3);
    P0 << 1.0, 0, 0,
          0,    1.0, 0,
          0,    0, 1.0;

    P = Eigen::MatrixXd(3,3);    

    Q = Eigen::MatrixXd(2,2);
    Q << 0.25, 0.0, 0.0, 0.5;
    
    xHat0 = Eigen::MatrixXd::Zero(3,1);

    // R = Eigen::MatrixXd(1,1);
    // R << 1.0;

    //Beta values for partial update
    betaVector = Eigen::MatrixXd(3,1);
    betaVector << 1.0, 1.0, 0.50;
}

Eigen::Matrix3d ZEstimator::quat_to_rot(std::vector<double> quat) {

    // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    // Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q[1], q[2], q[3]));
    // Eigen::Matrix3d rotation_matrix = I - 2 * q[0] * skew_mat + 2 * skew_mat * skew_mat;

    Eigen::Vector4d q;
    q << quat[0], quat[1], quat[2], quat[3];
    Eigen::Matrix3d R;
    
    // R(0,0) = s(0)*s(0)-s(1)*s(1)-s(2)*s(2)+s(3)*s(3);
    // R(0,1) = 2*(s(0)*s(1)-s(2)*s(3));
    // R(0,2) = 2*(s(0)*s(2)+s(1)*s(3));
    
    // R(1,0) = 2*(s(0)*s(1)+s(2)*s(3));
    // R(1,1) = -s(0)*s(0)+s(1)*s(1)-s(2)*s(2)+s(3)*s(3);
    // R(1,2) = 2*(s(1)*s(2)-s(0)*s(3));
    
    // R(2,0) = 2*(s(0)*s(2)-s(1)*s(3));
    // R(2,1) = 2*(s(1)*s(2)+s(0)*s(3));
    // R(2,2) = -s(0)*s(0)-s(1)*s(1)+s(2)*s(2)+s(3)*s(3);

    // First row of the rotation matrix
    R(0,0) = 2 * (q(0) * q(0) + q(1) * q(1)) - 1;
    R(0,1) = 2 * (q(1) * q(2) - q(0) * q(3));
    R(0,2) = 2 * (q(1) * q(3) + q(0) * q(2));
     
    // Second row of the rotation matrix
    R(1,0) = 2 * (q(1) * q(2) + q(0) * q(3));
    R(1,1) = 2 * (q(0) * q(0) + q(2) * q(2)) - 1;
    R(1,2) = 2 * (q(2) * q(3) - q(0) * q(1));
     
    // Third row of the rotation matrix
    R(2,0) = 2 * (q(1) * q(3) - q(0) * q(2));
    R(2,1) = 2 * (q(2) * q(3) + q(0) * q(1));
    R(2,2) = 2 * (q(0) * q(0) + q(3) * q(3)) - 1;

    return R;
}

void ZEstimator::initialize() {
    xHat = xHat0;
    P = P0;

    //Calculate partial update alphas
    int num_states = betaVector.rows();
    alphaVector = Eigen::MatrixXd(num_states, 1);
    gammas = Eigen::MatrixXd(num_states, num_states);

    //gammas = 1 - beta
    alphaVector = Eigen::MatrixXd::Ones(num_states, 1) - betaVector;
    gammas.setZero();
    gammas.diagonal() = alphaVector;
}

void ZEstimator::propagate(double ax, double ay, double az, std::vector<double> q, double dt) {
    Eigen::Matrix3d C_NED_to_body_frame = quat_to_rot(q);
    Eigen::Vector3d accelxyz_in_body_frame, accelxyz_in_NED_frame;
    accelxyz_in_body_frame << ax, ay, az;
    accelxyz_in_NED_frame = C_NED_to_body_frame * accelxyz_in_body_frame;

    u(0) = (accelxyz_in_NED_frame(2) - 1.0)*ONE_G; //assume normalization

    // std::cout << "Body = (" << accelxyz_in_body_frame(0) << ", " << accelxyz_in_body_frame(1) << ", " << accelxyz_in_body_frame(2) << ")" << std::endl;
    // std::cout << "Global = (" << accelxyz_in_NED_frame(0) << ", " << accelxyz_in_NED_frame(1) << ", " << accelxyz_in_NED_frame(2) << ")" << std::endl;
    // std::cout << u(0) << std::endl;

    F << 1, dt, 0,
         0, 1,  -dt,
         0, 0,  1;

    B << 0,
         1*dt,
         0;

    xHat = F*xHat + B*u;
    P = F*P*F.transpose() + G*Q*G.transpose();
}

void ZEstimator::update(double bz, double R_in) {
    R = Eigen::MatrixXd(1,1);
    R << R_in;
    
    z(0) = bz;

    Eigen::MatrixXd S;
    S = (H*P*H.transpose() + R);
    K = P*H.transpose()* S.inverse();

    xHat = xHat + K*(z - H*xHat);

    P = (I - K*H)*P;
}

//Partial update for states based on beta (%) parameters
//See "Partial-Update Schmidt–Kalman Filter" by Brink for details:
//https://arc.aiaa.org/doi/10.2514/1.G002808
void ZEstimator::partialUpdate(double bz, double R_in) {
    R = Eigen::MatrixXd(1,1);
    R << R_in;

    z(0) = bz;

    Eigen::MatrixXd S;
    S = (H*P*H.transpose() + R);
    K = P*H.transpose()* S.inverse();

    int numStates = xHat.rows();
    Eigen::MatrixXd xHatMinus(numStates, 1);
    Eigen::MatrixXd PMinus(numStates, numStates);
    Eigen::MatrixXd PPlus(numStates, numStates);

    //Save xHat and P minus values
    xHatMinus = xHat;
    PMinus = P;

    //Perform update
    xHat = xHat + K*(z - H*xHat);
    P = (I - K*H)*P;

    //Apply partial update
    PPlus = P;

    xHat = alphaVector.cwiseProduct(xHatMinus) + betaVector.cwiseProduct(xHat);
    P = gammas * (PMinus - PPlus) * gammas + PPlus;
}

void ZEstimator::reset() {
    //Reset covariance P
    P = P0;
    xHat = xHat0;
}
