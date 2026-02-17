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
  double roll = atan2(ay, az);
  double pitch = atan2(-ax, sqrt(ay * ay + az * az));
  euler_to_quaternion(roll, pitch, 0.0);
}

void Madgwick_Filter::euler_to_quaternion(double roll, double pitch, double yaw) {
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

void Madgwick_Filter::Madgwick_Update(double gx, double gy, double gz, double ax, double ay, double az) {
  double final_time = micros();
  t_interval = (final_time - init_time) / 1000000;
  init_time = final_time;

  double mag_accel = sqrtf(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  std::vector<double> a_I = {ax / mag_accel, ay / mag_accel, az / mag_accel};

  if (!quat_init_) {
    initialize_quaternion(a_I[0], a_I[1], a_I[2]);
    quat_init_ = true;
    return;
  }

  std::vector<double> quat_update_orig = update_quat(deg_to_rad(gx), deg_to_rad(gy), deg_to_rad(gz), a_I[0], a_I[1], a_I[2], q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);
  q_est_orig = {quat_update_orig[0], quat_update_orig[1], quat_update_orig[2], quat_update_orig[3]};
  euler_ = quaternion_to_euler(q_est_orig[0], q_est_orig[1], q_est_orig[2], q_est_orig[3]);
}

std::vector<double> Madgwick_Filter::update_quat(double gx, double gy, double gz, double ax, double ay, double az, double q0, double q1, double q2, double q3) {
  double beta = 0.75;
  double dt = t_interval;

  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

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

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = 1.0 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  return {q0, q1, q2, q3};
}
