#include "IMUTest.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IMUTest::IMUTest() : Node("imu_test_node") {
	RCLCPP_INFO(this->get_logger(), "IMU Test Node");

	wiringPiSetup();
	imu_.OPI_IMU_Setup();

	imu_msg_.header.stamp = this->get_clock()->now();

	imu_timer_ = this->create_wall_timer(10ms, std::bind(&IMUTest::imu_timer_callback, this));
}

void IMUTest::imu_timer_callback() {
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now-imu_msg_.header.stamp).seconds();

    imu_.IMU_read();
}

void IMUTest::baro_timer_callback() {

}

void IMUTest::baro_callback(const std_msgs::msg::Float64::SharedPtr msg) {
	
}