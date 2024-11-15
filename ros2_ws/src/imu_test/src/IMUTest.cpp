#include "IMUTest.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IMUTest::IMUTest() : Node("imu_test_node"), imu_init_(false), baro_init_(false) {
    RCLCPP_INFO(this->get_logger(), "IMU Test Node");

    wiringPiSetup();
    imu_.OPI_IMU_Setup();
    z_est_.initialize();

    imu_msg_.header.stamp = this->get_clock()->now();

    base_baro_sub_ = this->create_subscription<std_msgs::msg::Float64>("/Barometer/reading", 10, std::bind(&IMUTest::base_baro_callback, this, _1));
    cal_baro_sub_ = this->create_subscription<std_msgs::msg::Bool>("calibrate_barometer", 10, std::bind(&IMUTest::cal_baro_callback, this, _1));
    
    imu_timer_ = this->create_wall_timer(10ms, std::bind(&IMUTest::imu_timer_callback, this));
    baro_timer_ = this->create_wall_timer(100ms, std::bind(&IMUTest::baro_timer_callback, this));
}

void IMUTest::imu_timer_callback() {

    rclcpp::Time now = this->get_clock()->now();
    double dt = (now-imu_msg_.header.stamp).seconds();

    imu_.IMU_read();

    // RCLCPP_INFO(this->get_logger(), "a: (%.2f, %.2f, %.2f)", imu_.AccXraw, imu_.AccYraw, imu_.AccZraw);
    
    madgwick_.Madgwick_Update(imu_.gyr_rateXraw, imu_.gyr_rateYraw, imu_.gyr_rateZraw, imu_.AccXraw, imu_.AccYraw, imu_.AccZraw);

    //Get quaternion from madgwick
    std::vector<double> quat = madgwick_.get_quaternion();
    std::vector<double> euler = madgwick_.get_euler();

    if (imu_init_) {
        z_est_.propagate(imu_.AccXraw, imu_.AccYraw, imu_.AccZraw, quat, dt);
    } else {
        imu_init_ = true;
    }

    // RCLCPP_INFO(this->get_logger(), "R: %.2f, P: %.2f", euler[0], euler[1]);

    imu_msg_.header.stamp = now;

    //get orientation from madgwick
    // double pitch = madgwick_.pitch_final;
    // double roll = madgwick_.roll_final;
    // double yaw = madgwick_.yaw_final;

    // RCLCPP_INFO(this->get_logger(), "q: (%.2f, %.2f, %.2f, %.2f)", madgwick_.q1_, madgwick_.q2_, madgwick_.q3_, madgwick_.q4_);
}

void IMUTest::baro_timer_callback() {
    imu_.baro_read();

    if (!baro_init_) return;

    cal_baro_ = 44330 * (1 - pow(((imu_.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));

    z_est_.partialUpdate(cal_baro_);

    // RCLCPP_INFO(this->get_logger(), "Cal: %.2f", cal_baro_);
    // RCLCPP_INFO(this->get_logger(), "zHat: %.2f", z_est_.xHat(0));
}

void IMUTest::base_baro_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    base_baro_ = msg->data;
    baro_init_ = true;
}

void IMUTest::cal_baro_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    imu_.baro_read();

    //Set calibration offset
    baro_calibration_offset_ = imu_.comp_press - base_baro_;

    //Reset (zero) kalman filter
    z_est_.reset();
}


