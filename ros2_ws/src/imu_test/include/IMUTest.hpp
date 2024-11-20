#ifndef IMU_TEST_HPP
#define IMU_TEST_HPP

#include <rclcpp/rclcpp.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "OPI_IMU.hpp"
#include "ZEstimator.hpp"
#include "Madgwick_Filter.hpp"
#include "EMAFilter.hpp"

class IMUTest : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr imu_timer_, baro_timer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_velocity_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>:: SharedPtr baro_publisher_, temp_publisher_, diff_publisher_, alt_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr base_baro_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cal_baro_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped blimp_tf_;

    OPI_IMU imu_;
    ZEstimator z_est_;
    Madgwick_Filter madgwick_;

    sensor_msgs::msg::Imu imu_msg_;
    std_msgs::msg::Float64 z_msg_, z_vel_msg_;

    std_msgs::msg::Float64 baro_msg_, temp_msg_, diff_msg_;

    bool imu_init_, baro_init_;

    std::string blimp_name_;

    double base_baro_, baro_calibration_offset_, cal_baro_;

    double baro_sum_;
    int baro_count_;

    rclcpp::Time baro_time_;

    EMAFilter z_ema_;

    void imu_timer_callback();
    void baro_timer_callback();

    void base_baro_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void cal_baro_callback(const std_msgs::msg::Bool::SharedPtr msg);
public:
    IMUTest();
};

#endif