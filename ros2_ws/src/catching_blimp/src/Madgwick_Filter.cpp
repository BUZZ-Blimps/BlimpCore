/*
  Madgwick_Filter.cpp
*/

#include <iostream>
#include <vector>

#include <math.h>
#include <wiringPi.h>

#include "Madgwick_Filter.hpp"

Madgwick_Filter::Madgwick_Filter() : quat_init_(false) {
  init_time = micros();
}

std::vector<double> Madgwick_Filter::get_quaternion() {
  return q_est_orig;
}

std::vector<double> Madgwick_Filter::get_euler() {
  return euler_;
}

double Madgwick_Filter::deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

void Madgwick_Filter::initialize_quaternion(double ax, double ay, double az) {
    //First, initialize quaternion to just roll and pitch
    double roll = atan2(ay, az);
    double pitch = atan2(-ax, sqrt(ay * ay + az * az));

    std::cout << "RP: " << roll << "," << pitch << std::endl;
    euler_to_quaternion(roll, pitch, 0.0);
    // normalizeQuaternion();
    // quaternionToEuler();
}

void Madgwick_Filter::euler_to_quaternion(double roll, double pitch, double yaw) {
    //Must be radians!!
    q_est_orig[0] = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) - sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0); 
    q_est_orig[1] = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) + cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0); 
    q_est_orig[2] = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0) - sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0); 
    q_est_orig[3] = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0) + sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
}

std::vector<double> Madgwick_Filter::quaternion_to_euler(double q1, double q2, double q3, double q4) {
    double roll_rad = atan2f(q1 * q2 + q3 * q4, 0.5f - q2 * q2 - q3 * q3);
    double roll_deg = roll_rad * (180.0 / M_PI);
    double pitch_rad = asinf(-2.0f * (q2 * q4 - q1 * q3));
    double pitch_deg = pitch_rad * (180.0 / M_PI);
    double yaw_rad = atan2f(q2 * q3 + q1 * q4, 0.5f - q3 * q3 - q4 * q4);
    double yaw_deg = yaw_rad * (180.0 / M_PI);
    
    std::vector<double> angles_euler = {roll_deg, pitch_deg, yaw_deg};

    return angles_euler;
}

//Output
void Madgwick_Filter::Madgwick_Update(double gx, double gy, double gz, double ax, double ay, double az) {
  //Time Interval
  double final_time = micros();
  t_interval = (final_time - init_time) / 1000000; //in seconds
  init_time = final_time;

  //Gravity, gyro, and accel quaterions
  // std::vector<double> g_W = {0, 0, 0, 1}; //May need to make neg depending on orientation
  // std::vector<double> gyro_I = {gx * (M_PI / 180), gy * (M_PI / 180), gz * (M_PI / 180)}; // in rad/s(converted from deg/s)

  //std::vector<double> gyro_I = {0, -gy * (M_PI / 180), gx * (M_PI / 180), gz * (M_PI / 180)}; // in rad/s(converted from deg/s) for changed coordniates

  double mag_accel = sqrtf(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  std::vector<double> a_I = {ax / mag_accel, ay / mag_accel, az / mag_accel}; //Normalized Accel

  if (!quat_init_) {
      initialize_quaternion(a_I[0], a_I[1], a_I[2]);
      quat_init_ = true;
      return;
  }

  // std::cout << "acc: " << a_I[0] << "," << a_I[1] << "," << a_I[2] << std::endl;
  // initialize_quaternion(a_I[0], a_I[1], a_I[2]);

  //std::vector<double> a_I = {0, -ay / mag_accel, ax / mag_accel, az / mag_accel}; //Normalized Accel for changed cordinates

  std::vector<double> quat_update_orig = update_quat(deg_to_rad(gx), deg_to_rad(gy), deg_to_rad(gz), a_I[0], a_I[1], a_I[2], q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);
  // std::vector<double> angles_quat_update_g_lock = update_quat(gyro_I[2], gyro_I[1], -gyro_I[3], -a_I[2], -a_I[1], a_I[3], q_est_g_lock[0], q_est_g_lock[1], q_est_g_lock[2], q_est_g_lock[3]);

  q_est_orig = {quat_update_orig[0], quat_update_orig[1], quat_update_orig[2], quat_update_orig[3]};
  // q_est_g_lock = {angles_quat_update_g_lock[0], angles_quat_update_g_lock[1], angles_quat_update_g_lock[2], angles_quat_update_g_lock[3]};

  //Convert quat to euler angles for orig config
  euler_ = quaternion_to_euler(q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);

  // double roll_orig = angles_euler_orig_[0]; //converted to degrees //x-axis rot
  // double pitch_orig = angles_euler_orig_[1]; //converted to degrees //y-axis rot
  // double yaw_orig = angles_euler_orig_[2]; //converted to degrees   //z-axis rot

  //Convert quat to euler angles for g lock config
  // std::vector<double> angles_euler_g_lock = get_euler_angles_from_quat(q_est_g_lock[0], q_est_g_lock[1], q_est_g_lock[2], q_est_g_lock[3]);
  // double roll_g_lock = angles_euler_g_lock[0]; //converted to degrees //y-axis rot
  // double pitch_new = roll_g_lock;
  // double pitch_g_lock = angles_euler_g_lock[1]; //converted to degrees //x-axis rot
  // double roll_new = pitch_g_lock;
  // double yaw_g_lock = angles_euler_g_lock[2]; //converted to degrees   //z-axis rot
  // Stick with origional yaw (maybe try averaging it)

  //Fuse these to get the best of both configs for the final roll, pitch, and yaw
  // if (abs(pitch_orig) >= 45) {
  //   roll_final = roll_new;
  //   pitch_final = pitch_new;
  // }
  // else {
  //   roll_final = roll_orig;
  //   pitch_final = pitch_orig;
  // }

  // std::cout << "q: (" << q_est_orig[0] << ", " << q_est_orig[1] << ", " << q_est_orig[2] << ", " << q_est_orig[3] << ")" << std::endl;
  
  // q1_ = q_est_orig[0];
  // q2_ = q_est_orig[1];
  // q3_ = q_est_orig[2];
  // q4_ = q_est_orig[3];

  // yaw_final = yaw_orig;

  //  Serial.print(roll_orig);
  //  Serial.print(",");
  //  Serial.print(pitch_orig);
  //  Serial.print(",");
  //  Serial.println(yaw_orig);
  //  Serial.print(roll_g_lock);
  //  Serial.print(",");
  //  Serial.print(pitch_g_lock);
  //  Serial.print(",");
  //  Serial.println(yaw_g_lock);
  //  Serial.print(roll_final);
  //  Serial.print(",");
  //  Serial.print(pitch_final);
  //  Serial.print(",");
  //  Serial.println(yaw_final);
  //  Serial.print(q_est_orig[0]);
  //  Serial.print(",");
  //  Serial.print(q_est_orig[1]);
  //  Serial.print(",");
  //  Serial.print(q_est_orig[2]);
  //  Serial.print(",");
  //  Serial.println(q_est_orig[3]);
  //  Serial.print(q_est_g_lock[0]);
  //  Serial.print(",");
  //  Serial.print(q_est_g_lock[1]);
  //  Serial.print(",");
  //  Serial.print(q_est_g_lock[2]);
  //  Serial.print(",");
  //  Serial.println(q_est_g_lock[3]);
  //Serial.println("Next");

  //Prints frequency of the filter
  //Serial.println(1 / t_interval); //Frequency of the filter
  //delay(10);
}

std::vector<double> Madgwick_Filter::update_quat(double gx, double gy, double gz, double ax, double ay, double az, double q0, double q1, double q2, double q3) {

  // std::vector<double> q_est = {q1_est, q2_est, q3_est, q4_est};
  // std::vector<double> gyro_I = {0, Gyr_RateX, Gyr_RateY, Gyr_RateZ};
  // std::vector<double> a_I = {0, AccelX, AccelY, AccelZ};

  // //q_est components
  // double q1 = q_est[0];
  // double q2 = q_est[1];
  // double q3 = q_est[2];
  // double q4 = q_est[3];

  // // Auxiliary variables to avoid repeated arithmetic
  // double _2q1 = 2.0f * q1;
  // double _2q2 = 2.0f * q2;
  // double _2q3 = 2.0f * q3;
  // double _2q4 = 2.0f * q4;
  // double _4q1 = 4.0f * q1;
  // double _4q2 = 4.0f * q2;
  // double _4q3 = 4.0f * q3;
  // double _8q2 = 8.0f * q2;
  // double _8q3 = 8.0f * q3;
  // double q1q1 = q1 * q1;
  // double q2q2 = q2 * q2;
  // double q3q3 = q3 * q3;
  // double q4q4 = q4 * q4;

  // //Update Term
  // // Gradient decent algorithm corrective step (del_f)
  // double del_f1 = _4q1 * q3q3 + _2q3 * a_I[1] + _4q1 * q2q2 - _2q2 * a_I[2];
  // double del_f2 = _4q2 * q4q4 - _2q4 * a_I[1] + 4.0f * q1q1 * q2 - _2q1 * a_I[2] - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * a_I[3];
  // double del_f3 = 4.0f * q1q1 * q3 + _2q1 * a_I[1] + _4q3 * q4q4 - _2q4 * a_I[2] - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * a_I[3];
  // double del_f4 = 4.0f * q2q2 * q4 - _2q2 * a_I[1] + 4.0f * q3q3 * q4 - _2q3 * a_I[2];

  // //Tunable parameter
  // // double beta = .033; //from the original article
  // double beta = 0.5;

  // double del_f_norm = sqrtf(pow(del_f1, 2) + pow(del_f2, 2) + pow(del_f3, 2) + pow(del_f4, 2));
  // std::vector<double> del_q_est = { -beta*(del_f1 / del_f_norm),
  //                                  -beta*(del_f2 / del_f_norm),
  //                                  -beta*(del_f3 / del_f_norm),
  //                                  -beta*(del_f4 / del_f_norm)
  //                                }; //4x1 Matrix

  // //Orientation from Gyroscope
  // //quaternion product
  // // Rate of change of quaternion from gyroscope
  // std::vector<double> q_dot_w = {q_est[0]*gyro_I[0] - q_est[1]*gyro_I[1] - q_est[2]*gyro_I[2] - q_est[3]*gyro_I[3],
  //                               q_est[0]*gyro_I[1] + q_est[1]*gyro_I[0] + q_est[2]*gyro_I[3] - q_est[3]*gyro_I[2],
  //                               q_est[0]*gyro_I[2] - q_est[1]*gyro_I[3] + q_est[2]*gyro_I[0] + q_est[3]*gyro_I[1],
  //                               q_est[0]*gyro_I[3] + q_est[1]*gyro_I[2] - q_est[2]*gyro_I[1] + q_est[3]*gyro_I[0]
  //                              }; //4x1 Matrix

  // q_dot_w = {0.5f * q_dot_w[0],
  //            0.5f * q_dot_w[1],
  //            0.5f * q_dot_w[2],
  //            0.5f * q_dot_w[3]
  //           };

  // //Fuse Measurements
  // std::vector<double> q_est_dot = {q_dot_w[0] + del_q_est[0],
  //                                 q_dot_w[1] + del_q_est[1],
  //                                 q_dot_w[2] + del_q_est[2],
  //                                 q_dot_w[3] + del_q_est[3]
  //                                };

  // q_est = {q_est[0] + q_est_dot[0]*t_interval,
  //          q_est[1] + q_est_dot[1]*t_interval,
  //          q_est[2] + q_est_dot[2]*t_interval,
  //          q_est[3] + q_est_dot[3]*t_interval
  //         };  //Current estimate

  // double q_mag = sqrt(pow(q_est[0], 2) + pow(q_est[1], 2) + pow(q_est[2], 2) + pow(q_est[3], 2));
  // std::vector<double> q_norm = {q_est[0] / q_mag,
  //                               q_est[1] / q_mag,
  //                               q_est[2] / q_mag,
  //                               q_est[3] / q_mag
  //                              };//normalize the quaternion before next iteration
  // //will drift if not normalized

  // q_est = q_norm; //Update the previous estimate

    double beta = 0.75;
    double dt = t_interval;

  // std::vector<double> angles_quat_update = {q_est[0], q_est[1], q_est[2], q_est[3]};
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.0 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = 1.0 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    std::vector<double> quat_update = {q0, q1, q2, q3};

    return quat_update;
}


