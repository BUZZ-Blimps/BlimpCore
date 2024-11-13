#ifndef BARO_ACC_KF
#define BARO_ACC_KF

#include <Eigen/Dense>

class BaroAccKF {
  public:
  BaroAccKF();
  void predict(float dt);
  void updateBaro(float baro);
  void updateAccel(float acc);
  float x;
  float v;
  float a;
  float b;

  private:
  Eigen::Vector4f Xkp;
  Eigen::Matrix4f Pkp;
  Eigen::Matrix4f Qkp;

  float lastAccelTime = 0.0;
  
};

#endif