#ifndef IMU_TEST_HPP
#define IMU_TEST_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "OPI_IMU.hpp"
#include "ZEstimator.hpp"

class IMUTest : public rclcpp::Node {
private:
	rclcpp::TimerBase::SharedPtr imu_timer_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

	OPI_IMU imu_;
	ZEstimator z_est_;

	sensor_msgs::msg::Imu imu_msg_;

	void imu_callback();
	
public:
	IMUTest();
};

#endif