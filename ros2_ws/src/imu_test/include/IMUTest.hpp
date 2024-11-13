#ifndef IMU_TEST_HPP
#define IMU_TEST_HPP

#include <rclcpp/rclcpp.hpp>

#include "OPI_IMU.hpp"

class IMUTest : public rclcpp::Node {
private:
	OPI_IMU BerryIMU;
public:
	IMUTest();
}

#endif