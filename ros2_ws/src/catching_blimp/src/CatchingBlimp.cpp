#include "CatchingBlimp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// bool check = false;

//grabber data
int shoot = 0;
int grab = 0;
int shootCom = 0;
int grabCom = 0;

double searchYawDirection = -1;
double goalYawDirection = -1;

//avoidance data (9 quadrants), targets data and pixel data (balloon, orange goal, yellow goal)
//1000 means object is not present
std::vector<double> avoidance = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

bool check = false;

int counter = 0;
bool last_connected = false;
bool last_lost = true;

//Global variables
//sensor fusion objects
OPI_IMU BerryIMU;
TOF_Sense lidar;
Madgwick_Filter madgwick;

// MotorControl motorControl;
// Gimbal leftGimbal;
// Gimbal rightGimbal;
MotorControl_V2 motorControl_V2;

//Goal positioning controller
BangBang goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY); //Dead band, velocity to center itself

//filter on yaw gyro
EMAFilter yawRateFilter(0.2);
EMAFilter rollRateFilter(0.5);

//Low pass filter for computer vision parameters
EMAFilter xFilter(0.5);
EMAFilter yFilter(0.5);
EMAFilter zFilter(0.5);
EMAFilter theta_xFilter(0.5);
EMAFilter theta_yFilter(0.5);

// EMAFilter areaFilter(0.5);

//baro offset computation from base station value
// EMAFilter baroOffset(0.5);

//roll offset computation from imu
// EMAFilter rollOffset(0.5);

//ball grabber object
TripleBallGrabber ballGrabber;
// int counter = 0;
// bool last_connected = false;
// bool last_lost = true;

CatchingBlimp::CatchingBlimp() : 
    Node("catching_blimp_node"), 
    goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY),
    yawRateFilter(0.2),
    rollRateFilter(0.5),
    xFilter(0.5),
    yFilter(0.5),
    zFilter(0.5),
    theta_xFilter(0.5),
    theta_yFilter(0.5),
    count_(0), 
    target_detected_(false),
    target_id_(-1),
    target_type_(no_target),
    imu_init_(false), 
    baro_init_(false), 
    baro_calibration_offset_(0.0),
    baro_sum_(0.0), 
    baro_count_(0),
    z_hat_(0), 
    z_hat_2(0),
    catches_(0), 
    control_mode_(INITIAL_MODE), 
    auto_state_(searching),
    forward_command_(0),
    up_command_(0),
    yawrate_command_(0),
    rollrate_command_(0),
    roll_update_count_(0),
    theta_yPID_(1300, 0, 0){

    blimp_name_ = std::string(this->get_namespace()).substr(1);
    
    //Load PID config from params
    if (load_pid_config()) {
        RCLCPP_INFO(this->get_logger(), "PID configuration loaded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "PID configuration not provided. Exiting.");
        rclcpp::shutdown();
        return;
    }

    if (load_acc_calibration()) {
        RCLCPP_INFO(this->get_logger(), "Accelerometer calibration loaded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid accelerometer calibration. Exiting.");
        rclcpp::shutdown();
        return;
    }

    //Initialize TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    blimp_tf_.header.frame_id = "map";
    blimp_tf_.child_frame_id = blimp_name_;

    // initialize
    wiringPiSetup();
    BerryIMU.OPI_IMU_Setup();
    lidar.uart_setup();
    z_est_.initialize();
    z_est_2.initialize();
    z_lowpass_.setAlpha(0.1);
    z_lowpass_2.setAlpha(0.1);

    // targets_ = std::vector<double> {0.0, 0.0, 0.0};

    ballGrabber.ballgrabber_init(GATE_S, PIN_SCORING);
    motorControl_V2.motor_init(PIN_LEFT_UP, PIN_LEFT_FORWARD, PIN_RIGHT_UP, PIN_RIGHT_FORWARD, 25, 30, MIN_MOTOR, MAX_MOTOR);

    // Create publishers (7 right now)
    heartbeat_publisher = this->create_publisher<std_msgs::msg::Bool>("heartbeat", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    debug_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("debug", 10);
    height_publisher_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    z_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("z_velocity", 10);
    state_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("state", 10);
    log_publisher = this->create_publisher<std_msgs::msg::String>("log", 10);
    heading_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("heading", 10);

    // Set QOS settings to match basestation
    auto bool_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    bool_qos.reliable();
    bool_qos.transient_local();

    auto motor_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    motor_qos.reliable();
    motor_qos.durability_volatile();

    // Basestation bool subscribers
    auto_subscription = this->create_subscription<std_msgs::msg::Bool>("mode", bool_qos, std::bind(&CatchingBlimp::auto_subscription_callback, this, _1)); //was auto
    cal_baro_subscription = this->create_subscription<std_msgs::msg::Bool>("calibrate_barometer", bool_qos, std::bind(&CatchingBlimp::cal_baro_subscription_callback, this, _1));
    grabber_subscription = this->create_subscription<std_msgs::msg::Bool>("catching", bool_qos, std::bind(&CatchingBlimp::grab_subscription_callback, this, _1));
    shooter_subscription = this->create_subscription<std_msgs::msg::Bool>("shooting", bool_qos, std::bind(&CatchingBlimp::shoot_subscription_callback, this, _1));
    kill_subscription = this->create_subscription<std_msgs::msg::Bool>("killed", bool_qos, std::bind(&CatchingBlimp::kill_subscription_callback, this, _1));
    goal_color_subscription = this->create_subscription<std_msgs::msg::Bool>("goal_color", bool_qos, std::bind(&CatchingBlimp::goal_color_subscription_callback, this, _1));
    
    // Basestation motor commands
    motor_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("motor_commands", motor_qos, std::bind(&CatchingBlimp::motor_subscription_callback, this, _1)); 

    // Base barometer
    base_baro_subscription = this->create_subscription<std_msgs::msg::Float64>("/Barometer/reading", 10, std::bind(&CatchingBlimp::baro_subscription_callback, this, _1));

    // Offboard ML
    targets_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("targets", 10, std::bind(&CatchingBlimp::targets_subscription_callback, this, _1));

    // pixels_subscription = this->create_subscription<std_msgs::msg::Int64MultiArray>("pixels", 10, std::bind(&CatchingBlimp::pixels_subscription_callback, this, _1));
    avoidance_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("avoidance", 10, std::bind(&CatchingBlimp::avoidance_subscription_callback, this, _1));

    //100Hz IMU timer
    timer_imu = this->create_wall_timer(10ms, std::bind(&CatchingBlimp::imu_timer_callback, this));

    //25Hz barometer timer
    timer_baro = this->create_wall_timer(40ms, std::bind(&CatchingBlimp::baro_timer_callback, this));

    //33Hz state machine timer
    timer_state_machine = this->create_wall_timer(33ms, std::bind(&CatchingBlimp::state_machine_callback, this));

    //2Hz heartbeat timer
    timer_heartbeat = this->create_wall_timer(500ms, std::bind(&CatchingBlimp::heartbeat_timer_callback, this));

    //Initialize timestamps
    rclcpp::Time now = this->get_clock()->now();
    start_time_ = now;
    imu_msg_.header.stamp = now;
    state_machine_time_ = now;

    target_memory_time_ = now;
    last_catch_time_ = now;
    catch_start_time_ = now;
    caught_start_time_ = now;
    search_start_time_ = now;
    shoot_start_time_ = now;
    score_start_time_ = now;
    approach_start_time_ = now;
    goal_approach_start_time_ = now;

    state_machine_dt_ = 0;

    heartbeat_msg_.data = true;

    //Initialize state message
    state_msg_.data.reserve(2);
    state_msg_.data.push_back(0);
    state_msg_.data.push_back(0);

    debug_msg_.data.reserve(4);
    debug_msg_.data.push_back(0);
    debug_msg_.data.push_back(0);
    debug_msg_.data.push_back(0);
    debug_msg_.data.push_back(0);
}

void CatchingBlimp::heartbeat_timer_callback() {
    // Publish heartbeat to Basestation
    heartbeat_publisher->publish(heartbeat_msg_);

    // Publish autonomous state machine info to Basestation
    state_msg_.data[0] = auto_state_;
    state_msg_.data[1] = catches_;
    state_publisher_->publish(state_msg_);
}

void CatchingBlimp::imu_timer_callback() {
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - imu_msg_.header.stamp).seconds();
    
    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    lidar.TOF_read();
    
    //Apply IMU calibration
    Eigen::Vector3d acc_raw(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
    Eigen::Vector3d acc_cal = acc_A_*acc_raw - acc_b_;

    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, acc_cal(0), acc_cal(1), acc_cal(2));

    //Get quaternion from madgwick
    std::vector<double> quat = madgwick.get_quaternion();

    //Get euler angles from madgwick
    // std::vector<double> euler = madgwick_.get_euler();

    if (imu_init_) {
        //Only propagate after first IMU sample so dt makes sense
        z_est_.propagate(acc_cal(0), acc_cal(1), acc_cal(2), quat, dt);
        z_est_2.propagate(acc_cal(0), acc_cal(1), acc_cal(2), quat, dt);
    } else {
        imu_init_ = true;
    }

    imu_msg_.header.stamp = now;
    imu_msg_.orientation.w = quat[0];
    imu_msg_.orientation.x = quat[1];
    imu_msg_.orientation.y = quat[2];
    imu_msg_.orientation.z = quat[3];

    imu_msg_.angular_velocity.x = BerryIMU.gyr_rateXraw;
    imu_msg_.angular_velocity.y = BerryIMU.gyr_rateYraw;
    imu_msg_.angular_velocity.z = BerryIMU.gyr_rateZraw;

    imu_msg_.linear_acceleration.x = BerryIMU.AccXraw;
    imu_msg_.linear_acceleration.y = BerryIMU.AccYraw;
    imu_msg_.linear_acceleration.z = BerryIMU.AccZraw;

    imu_publisher_->publish(imu_msg_);

    
    //Lowpass propogated z estimate
    z_hat_ = z_lowpass_.filter(z_est_.xHat(0));
    z_hat_2 = z_lowpass_2.filter(z_est_2.xHat(0));

    z_msg_.data = z_hat_2;
    // z_msg_.data = double(lidar.dis)/1000;
    height_publisher_->publish(z_msg_);

    debug_msg_.data[0] = z_hat_; //baro and lidar kalman
    debug_msg_.data[1] = z_hat_2; //only baro kalman
    debug_msg_.data[2] = double(lidar.dis)/1000; //only lidar

    debug_publisher->publish(debug_msg_);

    z_vel_msg_.data = z_est_.xHat(1);
    z_velocity_publisher_->publish(z_vel_msg_);

    //Broadcast TF
    blimp_tf_.header.stamp = now;
    blimp_tf_.transform.translation.x = 0;
    blimp_tf_.transform.translation.y = 0;
    blimp_tf_.transform.translation.z = z_hat_;
    blimp_tf_.transform.rotation = imu_msg_.orientation;

    tf_broadcaster_->sendTransform(blimp_tf_);

    //update filtered yaw rate
    yawRateFilter.filter(BerryIMU.gyr_rateZraw);
    rollRateFilter.filter(BerryIMU.gyr_rateXraw);

    std::vector<double> euler_angles = madgwick.get_euler();
    double roll = euler_angles[0];

    std_msgs::msg::Float64MultiArray heading_msg_;
    heading_msg_.data = {euler_angles[2], BerryIMU.MagYraw, BerryIMU.MagXraw, std::atan2(BerryIMU.MagYraw, BerryIMU.MagXraw)*180/M_PI};
    heading_publisher_->publish(heading_msg_);

    //hyperbolic tan for yaw "filtering"
    double deadband = 1.0; // deadband for filteration
    yaw_motor_ = yawPID_.calculate(yawrate_command_, yawRateFilter.last, dt);
    if (fabs(yawrate_command_-yawRateFilter.last) < deadband) {
        yaw_motor_ = 0;
    }

    // else {
    //     yaw_motor_ = tanh(yaw_motor_)*fabs(yaw_motor_); // yaw for motor
    // }

    // Update roll controller every 4 timesteps
    if (roll_update_count_ == 4) {
        double deadband_roll = 5.0;
        rollrate_command_ = rollPID_.calculate(0, roll, dt);
        if (fabs(roll) < deadband_roll) {
            rollrate_command_ = 0;
        }

        roll_update_count_ = 0;
    }
    roll_update_count_++;

    double deadband_rollRate = 1.0;
    roll_rate_motor_ = rollRatePID_.calculate(rollrate_command_, rollRateFilter.last, dt);
    if (fabs(rollrate_command_ - rollRateFilter.last) < deadband_rollRate) {
        roll_rate_motor_ = 0;
    }

    if (MOTOR_PRINT_DEBUG) {
        RCLCPP_INFO(this->get_logger(), "F: %.2f, U: %.2f, Y: %.2f, R: %.2f", forward_motor_, up_motor_, yaw_motor_, roll_rate_motor_);
    }

    // Wait 5 seconds before doing anything interesting
    if ((now - start_time_).seconds() < 5.0) {
        // Zero motors while filters converge and esc arms
        motorControl_V2.update(0, 0, 0, 0);
        return;
    }

    // Different modes of motor activation
    if (ZERO_MODE) {
        motorControl_V2.update(0, 0, 0, 0);
        return;
    } else if (VERT_MODE && YAW_MODE) {
        motorControl_V2.update(0, up_motor_, yaw_motor_, 0);
        return;
    } else if (VERT_MODE) {
        motorControl_V2.update(0, up_motor_, 0, 0);
        return;
    } else if (YAW_MODE) {
        motorControl_V2.update(0, 0, yaw_motor_, 0);
        return;
    }

    motorControl_V2.update(forward_motor_, up_motor_, yaw_motor_, roll_rate_motor_);
}

void CatchingBlimp::baro_timer_callback() {

    // Get current barometer reading
    BerryIMU.baro_read();
    lidar.TOF_read();

    if (!baro_init_) return;

    cal_baro_ = 44330 * (1 - pow(((BerryIMU.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));
    baro_sum_ += cal_baro_;
    baro_count_++;

    //Average barometer every 5 samples (5Hz)
    if (baro_count_ == 5) {
        double baro_mean_ = baro_sum_/(double)baro_count_;
        debug_msg_.data[3] = baro_mean_;

        //rely on barometer data if drastic difference between barometer in lidar, likely because object is below blimp
        if(abs(baro_mean_ - R_lid) > 1.5){
            R_bar = 1.0;
            R_lid = 10.0;
        }
        
        z_est_.partialUpdate(baro_mean_, R_bar);
        z_est_.partialUpdate(double(lidar.dis)/1000, R_lid);

        z_est_2.partialUpdate(baro_mean_, R_bar);

        //Lowpass current estimate
        z_hat_ = z_lowpass_.filter(z_est_.xHat(0));

        z_hat_2 = z_lowpass_2.filter(z_est_2.xHat(0));

        baro_sum_ = 0.0;
        baro_count_ = 0;
    }
}


void CatchingBlimp::calculate_avoidance_from_quadrant(int quadrant){
    forward_avoidance_ = 0.0;
    up_avoidance_ = 0.0;
    yaw_avoidance_ = 0.0;

    //set avoidence command based on quadrant that contains object to avoid
    switch (quadrant) {
    case 1:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = -UP_AVOID;
        yaw_avoidance_ = -YAW_AVOID;
        break;
    case 2:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = -UP_AVOID;
        yaw_avoidance_ = 0;
        break;
    case 3:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = -UP_AVOID;
        yaw_avoidance_ = YAW_AVOID;
        break;
    case 4:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = 0;
        yaw_avoidance_ = -YAW_AVOID;
        break;
    case 5:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = 0;
        yaw_avoidance_ = 0;
        break;
    case 6:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = 0;
        yaw_avoidance_ = YAW_AVOID;
        break;
    case 7:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = UP_AVOID;
        yaw_avoidance_ = -YAW_AVOID;
        break;
    case 8:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = UP_AVOID;
        yaw_avoidance_ = 0;
        break;
    case 9:
        forward_avoidance_ = -FORWARD_AVOID;
        up_avoidance_ = UP_AVOID;
        yaw_avoidance_ = YAW_AVOID;
        break;
    default:
        break;
    }
}

std::string CatchingBlimp::auto_state_to_string(autoState state){
    std::string state_str;
    switch(state) {
        case searching: {
            state_str = std::string("searching");
            break;
        }
        case approach: {
            state_str = std::string("approach");
            break;
        }
        case catching: {
            state_str = std::string("catching");
            break;
        }
        case caught: {
            state_str = std::string("caught");
            break;
        }
        case goalSearch: {
            state_str = std::string("goalSearch");
            break;
        }
        case approachGoal: {
            state_str = std::string("approachGoal");
            break;
        }
        case scoringStart: {
            state_str = std::string("scoringStart");
            break;
        }
        case shooting: {
            state_str = std::string("shooting");
            break;
        }
        case scored: {
            state_str = std::string("score");
            break;
        } default: {
            state_str = std::string("invalid");
        }
    }
    return state_str;
}

target_type CatchingBlimp::auto_state_to_desired_target_type(autoState state){
    target_type desired_target_type;
    if(state == searching ||
       state == approach ||
       state == catching ||
       state == caught){
        desired_target_type = ball;
    }else if(state == goalSearch ||
             state == approachGoal ||
             state == scoringStart ||
             state == shooting ||
             state == scored){
        desired_target_type = goal;
    }else{
        desired_target_type = no_target;
    }
    return desired_target_type;
}

void CatchingBlimp::publish_log(std::string message) {
    auto log_msg = std_msgs::msg::String();
    log_msg.data = message;
    log_publisher->publish(log_msg);
    // RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

void CatchingBlimp::auto_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    if (msg->data) {
        if (control_mode_ == manual) {
            publish_log("Activating Auto Mode");
        }
        control_mode_ = autonomous;
    } else {
        if (control_mode_ == autonomous) {
            publish_log("Going Manual for a Bit...");
        }
        control_mode_ = manual;
    }

    //Reset autonomous state whenever control mode is changed
    auto_state_ = searching;
}

void CatchingBlimp::cal_baro_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    (void) msg;

    // Barometer Calibration
    // Read latest pressure value
    BerryIMU.baro_read();
    baro_calibration_offset_ = BerryIMU.comp_press - base_baro_;

    // Reset (zero) kalman filter
    z_est_.reset();

    publish_log(std::to_string(BerryIMU.comp_press));
    publish_log(std::to_string(base_baro_));
    publish_log(std::to_string(baro_calibration_offset_));

    publish_log("Calibrating Barometer");
}

void CatchingBlimp::baro_subscription_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: %.4f", msg->data);

    base_baro_ = msg->data;

    if (!baro_init_) {
        RCLCPP_INFO(this->get_logger(), "Base barometer initialized.");
        baro_init_ = true;
    }

    // //filter base station data
    // baroOffset.filter(baseBaro - BerryIMU.comp_press);

    //If teensy comes out of lost control mode, put it in manual control mode
    if (control_mode_ == lost) {
        control_mode_ = manual;
    }
}

void CatchingBlimp::grab_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (grabCom == 0 && msg->data) {
        grabCom = 1;
        publish_log("Going for a catch...");
    } else if (grabCom == 1 && !msg->data) {
        grabCom = 0;
        publish_log("Hopefully I got a balloon!");
    }
}

void CatchingBlimp::kill_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (msg->data == true) {
        publish_log("I'm ded xD");
        motorControl_V2.update(0,0,0,0);
    }
}

void CatchingBlimp::shoot_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (shootCom == 0 && msg->data) {
        shootCom = 1;
        publish_log("I'm shooting my shot...");
    } else if (shootCom == 1 && !msg->data) {
        shootCom = 0;
        publish_log("What a shot!");
    }
}

void CatchingBlimp::motor_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // Manual control commands from basestation
    forward_msg = msg->data[3];
    up_msg = msg->data[1];
    yaw_msg = msg->data[0];
}

void CatchingBlimp::goal_color_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    int goal_color = msg->data;
    if (goalColor != orange && goal_color == 0) {
        goalColor = orange;
        publish_log("Goal Color changed to Orange");
    } else if (goalColor != yellow && goal_color == 1) {
        goalColor = yellow;
        publish_log("Goal Color changed to Yellow");
    }
}

void CatchingBlimp::avoidance_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        avoidance[i] = msg->data[i];
    }
}

void CatchingBlimp::targets_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();

    // Check if the detection is valid (negative values mean nothing detected)
    if (msg->data[0] >= 0) {
        int new_id = static_cast<int>(msg->data[3]);
        target_type new_type = static_cast<target_type>(msg->data[4]);

        // Create a new detection (centering as before)
        TargetData new_detection;
        new_detection.x = msg->data[0] - 320;
        new_detection.y = msg->data[1] - 240;
        new_detection.z = msg->data[2];
        new_detection.timestamp = now;
        new_detection.id = new_id;
        new_detection.type = new_type;
        new_detection.theta_x = msg->data[5];
        new_detection.theta_y = msg->data[6];

        // Check desired target type
        target_type desired_target_type = auto_state_to_desired_target_type(auto_state_);

        // Check if previous target misaligns with desired target
        if(target_detected_ && (target_type_ != desired_target_type)){
            // Reset target
            target_detected_ = false;
            target_history_.clear();
            xFilter.reset();
            yFilter.reset();
            zFilter.reset();
            theta_xFilter.reset();
            theta_yFilter.reset();
        }

        // If we have not yet detected a target or the new detection matches our current target...
        if ((!target_detected_ && new_type == desired_target_type) || (new_id == target_id_ && new_type == target_type_)) {
            // Update the current target info
            target_detected_ = true;
            target_id_ = new_id;
            target_type_ = new_type;
            target_memory_time_ = now;

            // Add to the history buffer and keep only the latest 5 entries
            target_history_.push_back(new_detection);
            if (target_history_.size() > 10)
                target_history_.pop_front();

            // Update filtered target coordinates
            target_.x = xFilter.filter(new_detection.x);
            target_.y = yFilter.filter(new_detection.y);
            target_.z = zFilter.filter(new_detection.z);
            target_.theta_x = theta_xFilter.filter(new_detection.theta_x);
            target_.theta_y = theta_yFilter.filter(new_detection.theta_y);
        }
    }
}

void CatchingBlimp::vision_timer_callback() {
    rclcpp::Time now = this->get_clock()->now();

    if(target_detected_){
        // Check desired target type
        target_type desired_target_type = auto_state_to_desired_target_type(auto_state_);

        if((now - target_memory_time_).seconds() < TARGET_PREDICTION_USE_DELAY && target_type_ == desired_target_type){
            // Target is recent, use it!
        }else if((now - target_memory_time_).seconds() < TARGET_MEMORY_TIMEOUT  && target_type_ == desired_target_type){
            // Target isn't super recent, use prediction!
            TargetData predicted = predictTargetPosition();
            target_.x = predicted.x;
            target_.y = predicted.y;
            target_.z = predicted.z;
            target_.theta_x = predicted.theta_x;
            target_.theta_y = predicted.theta_y;
        }else{
            // Target is old, clear target and history
            RCLCPP_INFO(this->get_logger(), "LOST TRACKING ON OBJECT.");
            target_detected_ = false;
            target_history_.clear();
            xFilter.reset();
            yFilter.reset();
            zFilter.reset();
            theta_xFilter.reset();
            theta_yFilter.reset();
        }
    }

    // Optionally print or log the target’s state for debugging.
    if (VISION_PRINT_DEBUG) {
        if (target_detected_) {
            std::string target_str = (target_type_ == ball ? "Ball" : (target_type_ == goal ? "Goal" : "None"));
            RCLCPP_INFO(this->get_logger(), "Target: %s, ID: %d, Pos: (%.2f, %.2f, %.2f)",
                        target_str.c_str(), target_id_, target_.x, target_.y, target_.z);
        } else {
            RCLCPP_INFO(this->get_logger(), "No target detected.");
        }
    }
}


float CatchingBlimp::searchDirection() {
    int ran = rand()%10; //need to check bounds
    float binary = 1.0;
    if (ran <= 4){
        binary = 1.0;
    }else if (ran > 4){
        binary = -1.0;
    }
    return binary;
}

// //Auto PID control (output fed into manual controller)
// PID yPID_(0.8,0,0);       //TODO:retune these (can also be in pixels depends on which one performs better) 0.0075 for pixel PID
// PID xPID_(0.035,0,0);     //TODO:retune these 0.162 for pixel PID
// PID zPID_(350, 0, 0);     //not used for now due to baro reading malfunction
// PID yawPID_(12.0, 0, 0);  //can also tune kd with a little overshoot induced

bool CatchingBlimp::load_pid_config() {
    //PID gains
    this->declare_parameter("x_p", 0.0);
    this->declare_parameter("x_i", 0.0);
    this->declare_parameter("x_d", 0.0);
    this->declare_parameter("y_p", 0.0);
    this->declare_parameter("y_i", 0.0);
    this->declare_parameter("y_d", 0.0);
    this->declare_parameter("z_p", 0.0);
    this->declare_parameter("z_i", 0.0);
    this->declare_parameter("z_d", 0.0);
    this->declare_parameter("yaw_p", 0.0);
    this->declare_parameter("yaw_i", 0.0);
    this->declare_parameter("yaw_d", 0.0);
    this->declare_parameter("roll_p", 0.0);
    this->declare_parameter("roll_i", 0.0);
    this->declare_parameter("roll_d", 0.0);
    this->declare_parameter("rollRate_p", 0.0);
    this->declare_parameter("rollRate_i", 0.0);
    this->declare_parameter("rollRate_d", 0.0);
    
    double x_p, x_i, x_d, y_p, y_i, y_d, z_p, z_i, z_d, yaw_p, yaw_i, yaw_d, roll_p, roll_i, roll_d, rollRate_p, rollRate_i, rollRate_d;
    if (
        this->get_parameter("x_p", x_p) &&
        this->get_parameter("x_i", x_i) &&
        this->get_parameter("x_d", x_d) &&
        this->get_parameter("y_p", y_p) &&
        this->get_parameter("y_i", y_i) &&
        this->get_parameter("y_d", y_d) &&
        this->get_parameter("z_p", z_p) &&
        this->get_parameter("z_i", z_i) &&
        this->get_parameter("z_d", z_d) &&
        this->get_parameter("yaw_p", yaw_p) &&
        this->get_parameter("yaw_i", yaw_i) &&
        this->get_parameter("yaw_d", yaw_d) &&
        this->get_parameter("roll_p", roll_p) &&
        this->get_parameter("roll_i", roll_i) &&
        this->get_parameter("roll_d", roll_d) &&
        this->get_parameter("rollRate_p", rollRate_p) &&
        this->get_parameter("rollRate_i", rollRate_i) &&
        this->get_parameter("rollRate_d", rollRate_d) 
    ){
        //Set gains
        xPID_ = PID(x_p, x_i, x_d);  // left and right
        yPID_ = PID(y_p, y_i, y_d);  // up and down
        zPID_ = PID(z_p, z_i, z_d);  // unused
        yawPID_ = PID(yaw_p, yaw_i, yaw_d); // yaw correction
        rollPID_ = PID(roll_p, roll_i, roll_d);
        rollRatePID_ = PID(rollRate_p, rollRate_i, rollRate_d);

        RCLCPP_INFO(this->get_logger(), 
            "PID Gains: x: (p=%.2f, i=%.2f, d=%.2f), y: (p=%.2f, i=%.2f, d=%.2f), roll: (p=%.2f, i=%.2f, d=%.2f), rollrate: (p=%.2f, i=%.2f, d=%.2f), yawrate: (p=%.2f, i=%.2f, d=%.2f)", 
            x_p, x_i, x_d, y_p, y_i, y_d, roll_p, roll_i, roll_d, rollRate_p, rollRate_i, rollRate_d, yaw_p, yaw_i, yaw_d);

        return true;
    } else {
        return false;
    }
}

bool CatchingBlimp::load_acc_calibration() {
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

TargetData CatchingBlimp::predictTargetPosition(float offset) {
    TargetData predicted;

    // If we have fewer than two points, we cannot compute a velocity—return the last known value.
    if (target_history_.size() < 2) {
        predicted.x = target_.x;
        predicted.y = target_.y;
        predicted.z = target_.z;
        predicted.theta_x = target_.theta_x;
        predicted.theta_y = target_.theta_y;
        return predicted;
    }

    // Compute average velocity from the oldest to the most recent detection in the history.
    const TargetData& first = target_history_.front();
    const TargetData& last  = target_history_.back();
    double dt = (last.timestamp - first.timestamp).seconds();
    if (dt <= 0) {
        predicted.x = last.x;
        predicted.y = last.y;
        predicted.z = last.z;
        predicted.theta_x = target_.theta_x;
        predicted.theta_y = target_.theta_y;
        return predicted;
    }

    double vx = (last.x - first.x) / dt;
    double vy = (last.y - first.y) / dt;
    double vz = (last.z - first.z) / dt;
    double vtheta_x = (last.theta_x - first.theta_x) / dt;
    double vtheta_y = (last.theta_y - first.theta_y) / dt;

    // Determine the time difference between now and the last detection.
    double dt_pred = (this->get_clock()->now() - last.timestamp).seconds() + offset;

    // Predict the new position with a simple constant–velocity model:
    predicted.x = last.x + vx * dt_pred;
    predicted.y = last.y + vy * dt_pred;
    predicted.z = last.z + vz * dt_pred;
    predicted.theta_x = last.theta_x + vtheta_x * dt_pred;
    predicted.theta_y = last.theta_y + vtheta_y * dt_pred;
    return predicted;
}