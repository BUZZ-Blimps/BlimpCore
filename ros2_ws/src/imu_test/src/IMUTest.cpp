#include "IMUTest.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IMUTest::IMUTest() : Node("imu_test_node"), imu_init_(false), baro_init_(false), baro_sum_(0.0), baro_count_(0) {
    RCLCPP_INFO(this->get_logger(), "IMU Test node running");

    blimp_name_ = std::string(this->get_namespace()).substr(1);

    z_ema_.setAlpha(0.1);

    wiringPiSetup();
    imu_.OPI_IMU_Setup();
    z_est_.initialize();

    rclcpp::Time now = this->get_clock()->now();
    imu_msg_.header.stamp = now;
    baro_time_ = now;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    blimp_tf_.header.frame_id = "map";
    blimp_tf_.child_frame_id = blimp_name_;

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    height_publisher_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    z_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("z_velocity", 10);

    // base_baro_sub_ = this->create_subscription<std_msgs::msg::Float64>("/Barometer/reading", 10, std::bind(&IMUTest::base_baro_callback, this, _1));
    cal_baro_sub_ = this->create_subscription<std_msgs::msg::Bool>("calibrate_barometer", 10, std::bind(&IMUTest::cal_baro_callback, this, _1));
    
    imu_timer_ = this->create_wall_timer(10ms, std::bind(&IMUTest::imu_timer_callback, this));
    // baro_timer_ = this->create_wall_timer(10ms, std::bind(&IMUTest::baro_timer_callback, this));
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
        //Only propagate after first IMU sample so dt makes sense
        z_est_.propagate(imu_.AccXraw, imu_.AccYraw, imu_.AccZraw, quat, dt);
    } else {
        imu_init_ = true;
    }

    // RCLCPP_INFO(this->get_logger(), "R: %.2f, P: %.2f", euler[0], euler[1]);
    imu_msg_.header.stamp = now;
    imu_msg_.orientation.w = quat[0];
    imu_msg_.orientation.x = quat[1];
    imu_msg_.orientation.y = quat[2];
    imu_msg_.orientation.z = quat[3];

    imu_msg_.angular_velocity.x = imu_.gyr_rateXraw;
    imu_msg_.angular_velocity.y = imu_.gyr_rateYraw;
    imu_msg_.angular_velocity.z = imu_.gyr_rateZraw;

    imu_msg_.linear_acceleration.x = imu_.AccXraw;
    imu_msg_.linear_acceleration.y = imu_.AccYraw;
    imu_msg_.linear_acceleration.z = imu_.AccZraw;

    imu_publisher_->publish(imu_msg_);

    double z_hat = z_ema_.filter(z_est_.xHat(0));

    z_msg_.data = z_hat;
    height_publisher_->publish(z_msg_);

    z_vel_msg_.data = z_est_.xHat(1);
    z_velocity_publisher_->publish(z_vel_msg_);

    //Broadcast TF
    blimp_tf_.header.stamp = now;
    blimp_tf_.transform.translation.x = 0;
    blimp_tf_.transform.translation.y = 0;
    blimp_tf_.transform.translation.z = z_hat;
    blimp_tf_.transform.rotation = imu_msg_.orientation;

    tf_broadcaster_->sendTransform(blimp_tf_);

    imu_.baro_read();
    if (!baro_init_) {
        baro_init_ = true;
        baro_calibration_offset_ = 0;
        base_baro_ = imu_.comp_press;
        return;
    }

    cal_baro_ = 44330 * (1 - pow(((imu_.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));
    baro_sum_ += cal_baro_;
    baro_count_++;

    if ((now-baro_time_).seconds() >= 0.25) {

        double baro_mean_ = baro_sum_/baro_count_;
        
        z_est_.update(baro_mean_);
        // z_est_.partialUpdate(baro_mean_);

        baro_sum_ = 0.0;
        baro_count_ = 0;
        baro_time_ = now;
    }

    //get orientation from madgwick
    // double pitch = madgwick_.pitch_final;
    // double roll = madgwick_.roll_final;
    // double yaw = madgwick_.yaw_final;
    // RCLCPP_INFO(this->get_logger(), "q: (%.2f, %.2f, %.2f, %.2f)", madgwick_.q1_, madgwick_.q2_, madgwick_.q3_, madgwick_.q4_);
}

void IMUTest::baro_timer_callback() {
    // imu_.baro_read();

    // if (!baro_init_) return;

    // cal_baro_ = 44330 * (1 - pow(((imu_.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));

    // z_est_.partialUpdate(cal_baro_);

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


