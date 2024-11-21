#include "IMUTest.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IMUTest::IMUTest() : Node("imu_test_node"), imu_init_(false), baro_init_(false), baro_sum_(0.0), baro_count_(0), baro_calibration_offset_(0.0) {
    RCLCPP_INFO(this->get_logger(), "IMU Test node running");

    blimp_name_ = std::string(this->get_namespace()).substr(1);

    if (load_acc_calibration()) {
        RCLCPP_INFO(this->get_logger(), "Accelerometer calibration loaded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid accelerometer calibration. Exiting.");
        rclcpp::shutdown();
        return;
    }

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

    blimp_raw_tf_.header.frame_id = "map";
    blimp_raw_tf_.child_frame_id = blimp_name_ + "Raw";

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    height_publisher_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    z_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("z_velocity", 10);

    baro_publisher_ = this->create_publisher<std_msgs::msg::Float64>("barometer/reading", 10);
    temp_publisher_ = this->create_publisher<std_msgs::msg::Float64>("barometer/temperature", 10);
    diff_publisher_ = this->create_publisher<std_msgs::msg::Float64>("barometer/difference", 10);

    alt_publisher_ = this->create_publisher<std_msgs::msg::Float64>("barometer/altitude", 10);

    base_baro_sub_ = this->create_subscription<std_msgs::msg::Float64>("/Barometer/reading", 10, std::bind(&IMUTest::base_baro_callback, this, _1));
    cal_baro_sub_ = this->create_subscription<std_msgs::msg::Bool>("calibrate_barometer", 10, std::bind(&IMUTest::cal_baro_callback, this, _1));
    
    imu_timer_ = this->create_wall_timer(10ms, std::bind(&IMUTest::imu_timer_callback, this));

    //25Hz Baro timer
    baro_timer_ = this->create_wall_timer(40ms, std::bind(&IMUTest::baro_timer_callback, this));
}

bool IMUTest::load_acc_calibration() {
    std::vector<double> empty_vect, beta_vect;
    this->declare_parameter("betas", rclcpp::PARAMETER_DOUBLE_ARRAY);

    if (this->get_parameter("betas", beta_vect)) {
        if (beta_vect.size() == 9) {
            acc_A_ << beta_vect[0], beta_vect[1], beta_vect[2], 
                      beta_vect[1], beta_vect[3], beta_vect[4], 
                      beta_vect[2], beta_vect[4], beta_vect[5];
            acc_b_ << beta_vect[6], beta_vect[7], beta_vect[8];

            RCLCPP_INFO(this->get_logger(), 
                "Accelerometer calibration loaded: betas = (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", 
                beta_vect[0], beta_vect[1], beta_vect[2], beta_vect[3], beta_vect[4], beta_vect[5], beta_vect[6], beta_vect[7], beta_vect[8]);

            return true;
        } else {
            return false;
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Accelerometer calibration not provided - defaulting to identity.");
        acc_A_.setIdentity();
        acc_b_.setZero();
        return true;
    }
}

void IMUTest::imu_timer_callback() {

    rclcpp::Time now = this->get_clock()->now();
    double dt = (now-imu_msg_.header.stamp).seconds();
    imu_msg_.header.stamp = now;

    imu_.IMU_read();

    //Apply IMU calibration
    Eigen::Vector3d acc_raw(imu_.AccXraw, imu_.AccYraw, imu_.AccZraw);

    Eigen::Vector3d acc_cal = acc_A_*acc_raw - acc_b_;
    // RCLCPP_INFO(this->get_logger(), "a: (%.2f, %.2f, %.2f)", imu_.AccXraw, imu_.AccYraw, imu_.AccZraw);
    
    madgwick_.Madgwick_Update(imu_.gyr_rateXraw, imu_.gyr_rateYraw, imu_.gyr_rateZraw, acc_cal(0), acc_cal(1), acc_cal(2));

    //Get quaternion from madgwick
    std::vector<double> quat = madgwick_.get_quaternion();
    std::vector<double> euler = madgwick_.get_euler();

    if (imu_init_) {
        //Only propagate after first IMU sample so dt makes sense
        z_est_.propagate(acc_cal(0), acc_cal(1), acc_cal(2), quat, dt);
    } else {
        imu_init_ = true;
    }

    // std::cout << imu_.AccXraw << ", " << imu_.AccYraw << ", " << imu_.AccZraw << std::endl;

    // RCLCPP_INFO(this->get_logger(), "R: %.2f, P: %.2f", euler[0], euler[1]);
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
    blimp_tf_.transform.translation.z = z_est_.xHat(0);
    blimp_tf_.transform.rotation = imu_msg_.orientation;
    tf_broadcaster_->sendTransform(blimp_tf_);
    // std::cout << "predict: " << z_est_.xHat(0) << std::endl;

    // imu_.baro_read();
    // if (!baro_init_) {
    //     baro_init_ = true;
    //     // baro_calibration_offset_ = 0;
    //     // base_baro_ = imu_.comp_press;
    //     return;
    // }

    //get orientation from madgwick
    // double pitch = madgwick_.pitch_final;
    // double roll = madgwick_.roll_final;
    // double yaw = madgwick_.yaw_final;
    // RCLCPP_INFO(this->get_logger(), "q: (%.2f, %.2f, %.2f, %.2f)", madgwick_.q1_, madgwick_.q2_, madgwick_.q3_, madgwick_.q4_);
}

void IMUTest::baro_timer_callback() {
    rclcpp::Time now = this->get_clock()->now();

    imu_.baro_read();

    if (!baro_init_) return;

    // cal_baro_ = 44330 * (1 - pow(((imu_.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));
    // z_est_.partialUpdate(cal_baro_);
    // RCLCPP_INFO(this->get_logger(), "zHat: %.2f", z_est_.xHat(0));

    cal_baro_ = 44330 * (1 - pow(((imu_.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));
    baro_sum_ += cal_baro_;
    baro_count_++;

    // std::cout << "sum=" << baro_sum_ << ", count=" << baro_count_ << std::endl;

    // RCLCPP_INFO(this->get_logger(), "Cal: %.8f", imu_.comp_press);

    //Average barometer every 5 samples (5Hz)
    if (baro_count_ == 5) {
        double baro_mean_ = baro_sum_/baro_count_;
        
        // z_est_.update(baro_mean_);
        z_est_.partialUpdate(baro_mean_);
        z_ema_.filter(z_est_.xHat(0));

        baro_sum_ = 0.0;
        baro_count_ = 0;
    }

    baro_msg_.data = imu_.comp_press;
    temp_msg_.data = imu_.comp_temp;
    diff_msg_.data = imu_.comp_press-base_baro_;

    baro_publisher_->publish(baro_msg_);
    temp_publisher_->publish(temp_msg_);
    diff_publisher_->publish(diff_msg_);
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

