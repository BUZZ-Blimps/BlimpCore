/*
 Madgwick_Filter.h 
*/

#ifndef MADGWICK_FILTER_HPP
#define MADGWICK_FILTER_HPP

#include <vector>

class Madgwick_Filter
{
  public:
    Madgwick_Filter();
    void Madgwick_Update(double gyr_rateXraw, double gyr_rateYraw, double gyr_rateZraw, double AccXraw, double AccYraw, double AccZraw);
    std::vector<double> get_quaternion();
    std::vector<double> get_euler();
    double roll_final;
    double pitch_final;
    double yaw_final;

  private:
    double deg_to_rad(double deg);
    void euler_to_quaternion(double roll, double pitch, double yaw);
    void initialize_quaternion(double ax, double ay, double az);
    std::vector<double> update_quat(double Gyr_RateX, double Gyr_RateY, double Gyr_RateZ, double AccelX, double AccelY, double AccelZ, double q1_est, double q2_est, double q3_est, double q4_est);
    std::vector<double> get_euler_angles_from_quat(double q1, double q2, double q3, double q4);
    std::vector<double> q_est_orig = {1, 0, 0, 0}; //Assumed initial orientation of IMU
    std::vector<double> q_est_g_lock = {1, 0, 0, 0}; //Another orientation just to account for gymbol lock
    std::vector<double> angles_euler_orig_ = {0, 0, 0};
    double init_time;
    double t_interval;
    bool quat_init_;
};

#endif