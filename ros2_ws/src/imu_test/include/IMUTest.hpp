#ifndef IMU_TEST_HPP
#define IMU_TEST_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "OPI_IMU.hpp"
#include "ZEstimator.hpp"
#include "Madgwick_Filter.hpp"

class IMUTest : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr imu_timer_, baro_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr base_baro_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cal_baro_sub_;

    OPI_IMU imu_;
    ZEstimator z_est_;
    Madgwick_Filter madgwick_;

    sensor_msgs::msg::Imu imu_msg_;
    bool imu_init_;
    bool baro_init_;

    double base_baro_, baro_calibration_offset_, cal_baro_;

    void imu_timer_callback();
    void baro_timer_callback();

    void base_baro_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void cal_baro_callback(const std_msgs::msg::Bool::SharedPtr msg);
public:
    IMUTest();
};

#endif