#include "CatchingBlimp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//global variables


double loop_time;


//blimp game parameters
int blimpColor = BLIMP_COLOR;
int goalColor = GOAL_COLOR;

//timeout message time
double lastMsgTime = -1.0;

//msg for commands
float forward_msg = 0;
float yaw_msg = 0;
float up_msg = 0;
float translation_msg = 0;

float rotation = 180;
float ground_pressure = 0;

//timers for state machine
double searchingTimeStart = 0.0;
bool backingUp = false;
double searchTime = 30000;
double backupTime = 6000;

double approachTimeStart = 0;
double approachTimeMax = 10000;   //ms

double catchMemoryTimer = 0;
double catchTimeStart = 0;
double catchTime = 2300;        //ms

double caughtTimeStart = 0;
double caughtTime = 3000;       //ms

double scoreTimeStart = 0;
double scoreTime = 1000;        //ms

double shootingTimeStart = 0;
double shootingTime = 4500;//2500;     //ms

double scoredTimeStart = 0;
double scoredTime = 4500 ;//2500;       //ms

double firstMessageTime = 0.0;

double lastCatch = 0.0;

//grabber data
int shoot = 0;
int grab = 0;
int shootCom = 0;
int grabCom = 0;

//interupt pin setup at timing for long range ultrasonic
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
const u_int8_t interruptPin = 22;
double z_distance_m = 0;

//avoidence data
int quadrant = 10;

//base station calibrate baro
// bool calibrateBaro = false;

//base station baro offset
// double baroCalibrationOffset = 0.0;

//direction from last ball search
bool wasUp = true;

//corrected baro
// double z_hat_ = 0.0;

double searchYawDirection = -1;

double goalYawDirection = -1;

// char log_buf[BUFFER_LEN];

//avoidance data (9 quadrants), targets data and pixel data (balloon, orange goal, yellow goal)
//1000 means object is not present
std::vector<double> avoidance = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
// std::vector<double> targets = {1000.0, 1000.0, 1000.0};

std::vector<int64_t> pixels = {1000, 1000, 0, 1000, 1000, 0, 1000, 1000, 0};

//------------------MICRO ROS publishers/subscribers--------------
//ROS node

//message types: String Bool Float32 Float32 MultiArray
//message topics : /auto /baseBarometer /blimpID /grabbing /killed /motorCommands /shooting /identify /imu /goal_color /state_machine

//service response&request
// test_msgs__srv__BasicTypes_Response res;
// test_msgs__srv__BasicTypes_Request req;
bool check = false;

//boolean messages
// auto auto_msg = std_msgs::msg::Bool();
// auto grab_msg = std_msgs::msg::Bool();
// auto shoot_msg = std_msgs::msg::Bool();
// auto kill_msg = std_msgs::msg::Bool();
// auto calibration_msg = std_msgs::msg::Bool();

//int64 message
// auto goal_color_msg = std_msgs::msg::Int64();
// auto state_machine_msg = std_msgs::msg::Int64();

//float64 message
// auto baro_msg = std_msgs::msg::Float64();
// auto height_msg = std_msgs::msg::Float64();
// auto z_velocity_msg = std_msgs::msg::Float64();

//float64multiarray and int64multiarray messages
// auto motor_msg = std_msgs::msg::Float64MultiArray();
// auto debug_msg = std_msgs::msg::Float64MultiArray();
// auto avoidance_msg = std_msgs::msg::Float64MultiArray();
// auto targets_msg = std_msgs::msg::Float64MultiArray();
// auto pixels_msg = std_msgs::msg::Float64MultiArray();

//String messages
// auto log_msg = std_msgs::msg::String();

//sensor message
// auto imu_msg = sensor_msgs::msg::Imu();

int counter = 0;
bool last_connected = false;
bool last_lost = true;

//Global variables
//sensor fusion objects
servo Servo_L;
servo Servo_R;
brushless Brushless_L;
brushless Brushless_R;
OPI_IMU BerryIMU;
Madgwick_Filter madgwick;
AccelGCorrection accelGCorrection;

//NOT USED
// Optical_Flow Flow;
// Kalman_Filter_Tran_Vel_Est kal_vel;
// OpticalEKF xekf(DIST_CONSTANT, GYRO_X_CONSTANT, GYRO_YAW_CONSTANT);
// OpticalEKF yekf(DIST_CONSTANT, GYRO_Y_CONSTANT, 0);
// GyroEKF gyroEKF;

//Gimbal leftGimbal(yawPin, pitchPin, motorPin, newDeadband, newTurnOnCom, newMinCom, newMaxCom);
MotorControl motorControl;
Gimbal leftGimbal;
Gimbal rightGimbal;

//Manual PID control
// PID forwardPID(300, 0, 0);  //not used
// PID translationPID(300, 0, 0); //not used

//Goal positioning controller
BangBang goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY); //Dead band, velocity to center itself

//pre process for accel before vertical kalman filter
// EMAFilter verticalAccelFilter(0.05);

//filter on yaw gyro
EMAFilter yawRateFilter(0.2);
EMAFilter rollRateFilter(0.5);

//Low pass filter for computer vision parameters
EMAFilter xFilter(0.5);
EMAFilter yFilter(0.5);
EMAFilter zFilter(0.5);
// EMAFilter areaFilter(0.5);

//baro offset computation from base station value
// EMAFilter baroOffset(0.5);

//roll offset computation from imu
// EMAFilter rollOffset(0.5);

//ball grabber object
TripleBallGrabber ballGrabber;

//timing global variables for each update loop
double lastSensorFastLoopTick = 0.0;
double lastStateLoopTick = 0.0;
double lastBaroLoopTick = 0.0;
double lastOpticalLoopTick = 0.0;

CatchingBlimp::CatchingBlimp() : Node("catching_blimp_node"), 
    count_(0), imu_init_(false), baro_init_(false), z_hat_(0), catches_(0), control_mode_(lost), auto_state_(searching) {

    blimp_name_ = std::string(this->get_namespace()).substr(1);
    
    //Load PID config from params
    if (load_pid_config()) {
        
        RCLCPP_INFO(this->get_logger(), "PID configuration loaded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "PID configuration not provided. Exiting.");
        rclcpp::shutdown();
        return;
    }

    //Initialize TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    blimp_tf_.header.frame_id = "map";
    blimp_tf_.child_frame_id = blimp_name_;

    // Start Servos
    // Servo_L.servo_setup(0);
    // Servo_R.servo_setup(2);
    // Servo_L.servo_angle(0);
    // Servo_R.servo_angle(180);

    // initialize
    wiringPiSetup();
    BerryIMU.OPI_IMU_Setup();
    z_est_.initialize();
    z_lowpass_.setAlpha(0.1);

    targets_ = std::vector<double> {0.0, 0.0, 0.0};

    ballGrabber.ballgrabber_init(GATE_S, PWM_G);
    // leftGimbal.gimbal_init(0,2,5,25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

    // create publishers (7 right now)
    heartbeat_publisher = this->create_publisher<std_msgs::msg::Bool>("heartbeat", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    debug_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("debug", 10);
    height_publisher_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    z_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("z_velocity", 10);
    state_machine_publisher = this->create_publisher<std_msgs::msg::Int64>("state_machine", 10);
    log_publisher = this->create_publisher<std_msgs::msg::String>("log", 10);

    //create subscribers (10 right now)
    //Base station
    auto_subscription = this->create_subscription<std_msgs::msg::Bool>("mode", 10, std::bind(&CatchingBlimp::auto_subscription_callback, this, _1)); //was auto
    baseBarometer_subscription = this->create_subscription<std_msgs::msg::Float64>("/Barometer/reading", 10, std::bind(&CatchingBlimp::baro_subscription_callback, this, _1));
    calibrateBarometer_subscription = this->create_subscription<std_msgs::msg::Bool>("calibrate_barometer", 10, std::bind(&CatchingBlimp::calibrateBarometer_subscription_callback, this, _1));
    grabber_subscription = this->create_subscription<std_msgs::msg::Bool>("catching", 10, std::bind(&CatchingBlimp::grab_subscription_callback, this, _1));
    shooter_subscription = this->create_subscription<std_msgs::msg::Bool>("shooting", 10, std::bind(&CatchingBlimp::shoot_subscription_callback, this, _1));
    motor_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("motor_commands", 10, std::bind(&CatchingBlimp::motor_subscription_callback, this, _1)); 
    kill_subscription = this->create_subscription<std_msgs::msg::Bool>("killed", 10, std::bind(&CatchingBlimp::kill_subscription_callback, this, _1));
    goal_color_subscription = this->create_subscription<std_msgs::msg::Bool>("goal_color", 10, std::bind(&CatchingBlimp::goal_color_subscription_callback, this, _1));
    
    // Offboard ML
    targets_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("targets", 10, std::bind(&CatchingBlimp::targets_subscription_callback, this, _1));

    // targets_subscription = this->create_subscription<geometry_msgs::msg::Point>("/object_detection", 10, std::bind(&CatchingBlimp::targets_subscription_callback, this, _1));
    pixels_subscription = this->create_subscription<std_msgs::msg::Int64MultiArray>("pixels", 10, std::bind(&CatchingBlimp::pixels_subscription_callback, this, _1));
    avoidance_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("avoidance", 10, std::bind(&CatchingBlimp::avoidance_subscription_callback, this, _1));

    //100Hz IMU timer
    timer_imu = this->create_wall_timer(10ms, std::bind(&CatchingBlimp::imu_timer_callback, this));

    //25Hz barometer timer
    timer_baro = this->create_wall_timer(40ms, std::bind(&CatchingBlimp::baro_timer_callback, this));

    //33Hz state machine timer
    timer_state_machine = this->create_wall_timer(33ms, std::bind(&CatchingBlimp::state_machine_callback, this));

    //2Hz heartbeat timer
    timer_heartbeat = this->create_wall_timer(500ms, std::bind(&CatchingBlimp::heartbeat_timer_callback, this));

    //Start Brushless
    //     Brushless_L.brushless_setup(5);
    //     Brushless_R.brushless_setup(16);
    //     Brushless_L.brushless_thrust(1500);
    //     Brushless_R.brushless_thrust(1500);

    //Initialize timestamps
    firstMessageTime = micros()/MICROS_TO_SEC;
    imu_msg_.header.stamp = this->get_clock()->now();
    heartbeat_msg_.data = true;
}

void CatchingBlimp::heartbeat_timer_callback() {
    heartbeat_publisher->publish(heartbeat_msg_);
}

void CatchingBlimp::imu_timer_callback()
{
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now-imu_msg_.header.stamp).seconds();

    // float startTime = micros();
    loop_time = micros()/MICROS_TO_SEC;
    
    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    // BerryIMU.IMU_ROTATION(rotation); // Rotate IMU

    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);

    //Get quaternion from madgwick
    std::vector<double> quat = madgwick.get_quaternion();

    //Get euler angles from madgwick
    // std::vector<double> euler = madgwick_.get_euler();

    if (imu_init_) {
        //Only propagate after first IMU sample so dt makes sense
        z_est_.propagate(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, quat, dt);
    } else {
        imu_init_ = true;
    }

    // RCLCPP_INFO(this->get_logger(), "R: %.2f, P: %.2f", euler[0], euler[1]);
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

    z_msg_.data = z_hat_;
    height_publisher_->publish(z_msg_);

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
}

void CatchingBlimp::baro_timer_callback()
{
    // float startTime = micros();
    // auto height_msg = std_msgs::msg::Float64();
    // auto z_velocity_msg = std_msgs::msg::Float64();

    // get most current barometer values
    BerryIMU.baro_read();

    if (!baro_init_) return;

    cal_baro_ = 44330 * (1 - pow(((BerryIMU.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));

    //Update Z estimator with barometer measurement
    z_est_.partialUpdate(cal_baro_);

    //Lowpass current estimate
    z_hat_ = z_lowpass_.filter(z_est_.xHat(0));
}

void CatchingBlimp::state_machine_callback()
{
    // float startTime = micros();
    auto state_machine_msg = std_msgs::msg::Int64();
    auto debug_msg = std_msgs::msg::Float64MultiArray();
    debug_msg.data.resize(14);
    // double dt = 33/1000; // check this!! make sure its 30Hz
    double dt = 0.0; // dt=0  =>  PID->P

    debug_msg.data[9] = dt;

    // publish_log("Im in state_machine_callback");
    //control inputs
    float forwardCom = 0.0;
    float upCom = 0.0;
    float yawCom = 0.0;
    float translationCom = 0.0;

    //object avoidence commands to overide search and computer vition
    float forwardA = 0.0;
    float upA = 0.0;
    float yawA = 0.0;

    //set avoidence command based on quadrant that contains object to avoid
    switch (quadrant) {
    case 1:
        forwardA = -FORWARD_AVOID;
        upA = -UP_AVOID;
        yawA = -YAW_AVOID;
        break;
    case 2:
        forwardA = -FORWARD_AVOID;
        upA = -UP_AVOID;
        yawA = 0;
        break;
    case 3:
        forwardA = -FORWARD_AVOID;
        upA = -UP_AVOID;
        yawA = YAW_AVOID;
        break;
    case 4:
        forwardA = -FORWARD_AVOID;
        upA = 0;
        yawA = -YAW_AVOID;
        break;
    case 5:
        forwardA = -FORWARD_AVOID;
        upA = 0;
        yawA = 0;
        break;
    case 6:
        forwardA = -FORWARD_AVOID;
        upA = 0;
        yawA = YAW_AVOID;
        break;
    case 7:
        forwardA = -FORWARD_AVOID;
        upA = UP_AVOID;
        yawA = -YAW_AVOID;
        break;
    case 8:
        forwardA = -FORWARD_AVOID;
        upA = UP_AVOID;
        yawA = 0;
        break;
    case 9:
        forwardA = -FORWARD_AVOID;
        upA = UP_AVOID;
        yawA = YAW_AVOID;
        break;
    default:
        break;
    }

    //from base station
    //compute control mode machine
    if (control_mode_ == manual) {
        // publish_log("Im in state_machine_callback, manual");
        //get manual data
        //all motor commands are between -1 and 1
        //set max yaw command to 120 deg/s

        yawCom = -yaw_msg*120;

        if (USE_EST_VELOCITY_IN_MANUAL) {
            //set max velocities 2 m/s
            upCom = up_msg*2.0;
            forwardCom = forward_msg*2.0;
            translationCom = translation_msg*2.0;
        } else {
            //normal mapping using max esc command 
            // upCom = up_msg*2.0; //PID used and maxed out at 2m/s
            upCom = -up_msg*500.0; //up is negative
            // upCom = -up_msg*500.0-0.5*pitch; //pitch correction? (pitch in degrees, conversion factor command/degree)
            forwardCom = forward_msg*500.0;
            translationCom = translation_msg*500.0;
        }    

        //check if shooting should be engaged
        //this block switches the control mode to the oposite that it is currently in
        if (shoot != shootCom) {
            shoot = shootCom;

            //change shoot control mode
            if (ballGrabber.state == 2) {
                //stop shooting
                ballGrabber.closeGrabber(control_mode_);
            } else {
                //reset catch counter
                catches_ = 0;

                //go back to searching
                // auto_state_ = searching;
                // searching timer
                searchingTimeStart = millis();
                searchYawDirection = searchDirection();  //randomize the search direction

                //start shooting
                ballGrabber.shoot(control_mode_);
            }
        //check if grabbing should be engaged
        //this block switches the control mode to the oposite that it is currently in
        } else if (grab != grabCom) {
            grab = grabCom;

            //change grab control mode
            if (ballGrabber.state == 0) {
                ballGrabber.openGrabber(control_mode_);
            } else {
                ballGrabber.closeGrabber(control_mode_);

                //increase catch counter
                catches_++;

                //start catch timmer
                if (catches_ >= 1) {
                    lastCatch = micros()/MICROS_TO_SEC;
                }
            }
        }
    } else if (control_mode_ == autonomous) {
        //filter target data
        // float tx = 0;
        // float ty = 0;
        double tx = 0;
        double ty = 0;
        double tz = 0;
        double temp_tx = 0;
        double temp_ty = 0;
        // float area = 0;

        //new target (empty target)
        std::vector<double> detected_target;

        //if a target is seen
        if (targets_[0] != 0) {
            double rawZ = targets_[2]; // distance
            tx = xFilter.filter(static_cast<double>(targets_[0]));
            ty = yFilter.filter(static_cast<double>(targets_[1]));
            tz = zFilter.filter(rawZ);
            
            // tx = static_cast<float>(targets_[0]);
            // ty = static_cast<float>(targets_[1]);
            // tz = rawZ;

            detected_target.push_back(tx);
            detected_target.push_back(ty);
            detected_target.push_back(tz);
            
        } else {
            // no target, set to default value
            xFilter.filter(GAME_BaLL_X_OFFSET);
            yFilter.filter(GAME_BaLL_X_OFFSET);
            zFilter.filter(4);

            // areaFilter.filter(0);
        }

        /*
        //update targets data if any target exists
        //TODO: add area to verify distance 
        //balloon 
        if (targets[2] != 1000 && (auto_state_ == searching || auto_state_ == approach || auto_state_ == catching)) {
            float rawZ = targets[2]; //balloon distance
            // update filtered target coordinates (3D space, with center of camera as (0,0,0))
            // tx = xFilter.filter(targets[0]); (3D)
            // ty = yFilter.filter(targets[1]);
            tx = xFilter.filter(static_cast<float>(pixels[0]));
            ty = yFilter.filter(static_cast<float>(pixels[1]));
            tz = zFilter.filter(rawZ);
            // area = areaFilter.filter(target[0][3]);
            detected_target.push_back(tx);
            detected_target.push_back(ty);
            detected_target.push_back(tz);
        } else {
            //no target, set to default value
            xFilter.filter(0);
            yFilter.filter(0);
            zFilter.filter(0);
            // areaFilter.filter(0);
        }

        //orange goal
        //in goal scoring stages 
        if (targets[5] != 1000 && goalColor == orange && (auto_state_ == goalSearch || auto_state_ == approachGoal || auto_state_ == scoringStart)) {
            float rawZ = targets[5]; //balloon distance
            //update filtered target coordinates (3D space, with center of camera as (0,0,0))
            // tx = xFilter.filter(targets[3]);
            // ty = yFilter.filter(targets[4]); 
            tx = xFilter.filter(static_cast<float>(pixels[3]));
            ty = yFilter.filter(static_cast<float>(pixels[4])); 
            tz = zFilter.filter(rawZ);
            // area = areaFilter.filter(target[0][3]);
            detected_target.push_back(tx);
            detected_target.push_back(ty);
            detected_target.push_back(tz);
        } else {
            //no target, set to default value
            xFilter.filter(0);
            yFilter.filter(0);
            zFilter.filter(0);
            // areaFilter.filter(0);
        }

        //yellow goal
        if (targets[8] != 1000 && goalColor == yellow && (auto_state_ == goalSearch || auto_state_ == approachGoal || auto_state_ == scoringStart)) {
            float rawZ = targets[8]; //balloon distance
            //update filtered target coordinates (3D space, with center of camera as (0,0,0))
            // tx = xFilter.filter(targets[6]);
            // ty = yFilter.filter(targets[7]);
            tx = xFilter.filter(static_cast<float>(pixels[6]));
            ty = yFilter.filter(static_cast<float>(pixels[7]));
            tz = zFilter.filter(rawZ);
            // area = areaFilter.filter(target[0][3]);
            detected_target.push_back(tx);
            detected_target.push_back(ty);
            detected_target.push_back(tz);
        } else {
            //no target, set to default value
            xFilter.filter(0);
            yFilter.filter(0);
            zFilter.filter(0);
            // areaFilter.filter(0);
        }
        */

        //test target message
        // if (target.size() != 0){
        // debug_msg.data.data[0] = target[0];
        // debug_msg.data.data[1] = target[1];
        // debug_msg.data.data[2] = target[2];
        // debug_msg.data.size = 3;
        // }

        // RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

        //modes for autonomous behavior
        switch (auto_state_) {
            //autonomous state machine
            case searching: {

                std::cout << "searching..." << std::endl;

                //check if goal scoring should be attempted
                if (catches_ >= 1 && ((micros()/MICROS_TO_SEC - lastCatch) >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches_-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
                    catches_ = TOTAL_ATTEMPTS;
                    auto_state_ = goalSearch;
                    goalYawDirection = searchDirection();  //randomize search direction
                    break;
                }

                if (catches_ >= TOTAL_ATTEMPTS) {
                    auto_state_ = goalSearch;
                    goalYawDirection = searchDirection();  //randomize search direction
                    break;
                }

                std::cout << "checked catches" << std::endl;

                //begin search pattern spinning around at different heights
                if (detected_target.size() == 0) {

                    //keep ball grabber closed
                    // ballGrabber.closeGrabber();

                    //use object avoidence
                    double avoidanceMinVal = 1000.0; // Initialize 
                    int avoidanceMinIndex = 10;

                    // Iterate through the vector to find the minimum value and its index
                    // find the minimum distance and its corresponding quadrant number (1-9)
                    for (int i = 0; i < 9; ++i) {
                        if (avoidance[i] < avoidanceMinVal) {
                            avoidanceMinVal = avoidance[i]; //distance
                            avoidanceMinIndex = i+1; //quadrant number
                        }
                    }

                    //set the avoidance quadrant only when avoidance range is triggered
                    if (avoidanceMinVal < AVOID_TRIGGER){
                        //update quadrant
                        quadrant = avoidanceMinIndex;
                    } else {
                        //safe
                        //update quadrant
                        quadrant = 10;
                    }

                    //avoding obstacle
                    if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {

                        //overide search commands
                        yawCom = yawA;
                        forwardCom = forwardA;
                        upCom = upA;

                    } else {
                        //search behavior (no detected_target)
                        //spin in a small circle looking for a game ball
                        //randomize the diretion selection
                        // yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //upCom = 0;    //is overriden later, defined here as a safety net
                        // forwardCom = GAME_BALL_FORWARD_SEARCH;
                        
                        // Timeline:
                        // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
                        double elapsedSearchingTime = millis() - searchingTimeStart;

                        std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";
                        if (elapsedSearchingTime < searchTime) {
                            backingUp = false;
                            yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            forwardCom = GAME_BALL_FORWARD_SEARCH;

                            if (z_hat_ > CEIL_HEIGHT) {
                                upCom = GAME_BALL_VERTICAL_SEARCH; //down
                            }else if (z_hat_ < FLOOR_HEIGHT) {
                                upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                            }else{
                                upCom = 0;
                            }
                        } else if (elapsedSearchingTime < searchTime + backupTime) {
                            if (!backingUp) {
                                backingUp = true;
                                searchYawDirection = searchDirection();
                            }   
                            message += " Backup!";
                            
                            yawCom = 35*searchYawDirection;
                            forwardCom = -240;
                            upCom = -100;
                        }else{
                            message += " Reset!";
                            searchingTimeStart = millis();
                        }
                        publish_log(message);

                        // Prolly hit a net
                        // if (20000 < millis()- searchingTimeStart){
                        //     double backupTimer = millis();

                        //     std::string message = "20+ seconds.";

                        //     if (2200 > millis() - backupTimer) {
                        //         message += " first 2 seconds.";
                        //         searchYawDirection = searchDirection();
                        //         yawCom = 60*searchYawDirection;
                        //         upCom = -50;    //is overriden later, defined here as a safety net
                        //         forwardCom = -200;
                        //     }

                        //     searchingTimeStart = millis(); 
                        //     publish_log(message.c_str());
                        // }

                        //move up and down within the set boundry
                        // if (z_hat_ > CEIL_HEIGHT) {
                        //     // if (wasUp) wasUp = false;
                        //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     upCom = GAME_BALL_VERTICAL_SEARCH; //down
                        //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                        // }

                        // if (z_hat_ < FLOOR_HEIGHT) {
                        //     // if (!wasUp) wasUp = true;
                        //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                        //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                        // }

                        // if (z_hat_ <= CEIL_HEIGHT && z_hat_ >=FLOOR_HEIGHT) {
                        //     // if (wasUp) wasUp = false;
                        //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     upCom = 0;
                        //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                        // }
                    }

                } else {
                    //move to approaching game ball
                    auto_state_ = approach;
                    //start approaching timer
                    approachTimeStart = millis();
                }
                break;
            } case approach: {
                // max time to approach
                if (approachTimeMax < millis() - approachTimeStart) {
                    // auto_state_ = searching;
                    // searching timer
                    searchingTimeStart = millis();
                }

                //check if target is still valid
                if (detected_target.size() > 0) {
                    //seeing a target
                    //add memory
                    temp_tx = tx;
                    temp_ty = ty;
                    catchMemoryTimer = millis();

                    if (catches_ >= 1 && micros()/MICROS_TO_SEC - lastCatch >= MAX_SEARCH_WAIT_AFTER_ONE - (catches_-1)*(GAME_BALL_WAIT_TIME_PENALTY)) {
                      catches_ = TOTAL_ATTEMPTS;
                      auto_state_ = goalSearch;
                      goalYawDirection = searchDirection();  //randomize search direction
                      break;
                    }

                    if (catches_ >= TOTAL_ATTEMPTS) {
                      auto_state_ = goalSearch;
                      goalYawDirection = searchDirection();  //randomize search direction
                    }

                    //move toward the balloon
                    yawCom = xPID_.calculate(GAME_BaLL_X_OFFSET, tx, dt/1000); 
                    // yawCom = yawCom * -1;
                    // yawCom = (yawCom / (xPID_._kp * GAME_BaLL_X_OFFSET))*120;

                    debug_msg.data[0] = tx;
                    debug_msg.data[1] = ty;
                    // debug_msg.data[2] = yawCom;
                    upCom = -yPID_.calculate(GAME_BALL_APPROACH_ANGLE, ty, dt/1000);  

                    // upCom = (upCom / (yPID_._kp * GAME_BaLL_X_OFFSET))*500;

                    forwardCom = GAME_BALL_CLOSURE_COM;
                    translationCom = 0;

                    //check if the gate should be opened
                    if (tz < BALL_GATE_OPEN_TRIGGER) {
                        ballGrabber.openGrabber(control_mode_);

                        //check if the catching mode should be triggered
                        if (tz < BALL_CATCH_TRIGGER) {
                            // auto_state_ = catching;

                            //start catching timer
                            catchTimeStart = millis();

                            //turn motors off
                            motorControl.update(0,0,0,0,0);
                        }
                    } else {
                        //make sure grabber is closed, no game ball is close enough to catch
                        ballGrabber.closeGrabber(control_mode_);
                    }

                    //if target is lost within 1 second
                    //remember the previous info about where the ball is 
                }
                else if ((millis()-catchMemoryTimer) < 1000 && detected_target.size() == 0) {
                        yawCom = xPID_.calculate(GAME_BaLL_X_OFFSET, temp_tx, dt/1000); 
                        // yawCom = (yawCom / (xPID_._kp * GAME_BaLL_X_OFFSET))*120;
                        upCom = -yPID_.calculate(GAME_BALL_APPROACH_ANGLE, temp_ty, dt/1000);
                        // upCom = (upCom / (yPID_._kp * GAME_BaLL_X_OFFSET))*500;  
                        forwardCom = GAME_BALL_CLOSURE_COM;
                        translationCom = 0;
                } 
                // after two seconds of losing the target, the target is still not detected
                else {
                    //no target, look for another
                    //maybe add some memory
                    auto_state_ = searching;
                    // searching timer
                    searchingTimeStart = millis();
                    ballGrabber.closeGrabber(control_mode_);
                    searchYawDirection = searchDirection();  //randomize the search direction
                }

                break;
            } case catching: {
                //wait for 0.3 second
                // delay(300);

                forwardCom = CATCHING_FORWARD_COM;
                upCom = -CATCHING_UP_COM;
                yawCom = 0;
                translationCom = 0;

                if (catchTimeStart < millis() - catchTime) {
                    //catching ended, start caught timer
                    auto_state_ = caught;
                    caughtTimeStart = millis();
                    ballGrabber.closeGrabber(control_mode_);

                    //increment number of catches_
                    catches_ = catches_ + 1;

                    //start catch timmer
                    lastCatch = micros()/MICROS_TO_SEC;
                }
                break;
            } case caught: {
                if (catches_ > 0) {
                    //if a target is seen right after the catch
                    if (detected_target.size() > 0) {
                        //approach next game ball if visible
                        if (catches_ < TOTAL_ATTEMPTS) {
                            auto_state_ = searching;
                            // searching timer
                            searchingTimeStart = millis();
                            searchYawDirection = searchDirection();  //randomize the search direction
                        }
                    }

                    //decide if the blimp is going to game ball search or goal search
                    if (caughtTimeStart < millis() - caughtTime) {
                        if (catches_ >= TOTAL_ATTEMPTS) {
                            auto_state_ = goalSearch;
                            goalYawDirection = searchDirection();  //randomize search direction
                        } else {
                            auto_state_ = searching;
                            // searching timer
                            searchingTimeStart = millis();
                            searchYawDirection = searchDirection();  //randomize the search direction
                        }
                    }

                    forwardCom = CAUGHT_FORWARD_COM;
                    upCom = -CAUGHT_UP_COM;
                    yawCom = 0;
                } else {
                    auto_state_ = searching;
                    // searching timer
                    searchingTimeStart = millis();
                    searchYawDirection = searchDirection();  //randomize the search direction
                }
                break;
            } case goalSearch: {

                //keep ball grabber closed
                // ballGrabber.closeGrabber();

                //use object avoidence
                double avoidanceMinVal = 1000.0; // Initialize 
                int avoidanceMinIndex = 10;

                // Iterate through the vector to find the minimum value and its index
                // find the minimum distance and its corresponding quadrant number (1-9)
                for (int i = 0; i < 9; ++i) {
                    if (avoidance[i] < avoidanceMinVal) {
                        avoidanceMinVal = avoidance[i]; //distance
                        avoidanceMinIndex = i+1; //quadrant number
                    }
                }

                //set the avoidance quadrant only in range
                if (avoidanceMinVal < AVOID_TRIGGER) {
                    //update quadrant
                    quadrant = avoidanceMinIndex;
                } else {
                    //update quadrant
                    //safe
                    quadrant = 10;
                }

                if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {
                    //avoding obstacle
                    //overide search commands
                    yawCom = yawA;
                    forwardCom = forwardA;
                    upCom = upA;
                } else {
                    //goal search behavior
                    //randomize the diretion selection
                    yawCom = GOAL_YAW_SEARCH*goalYawDirection;
                    upCom = -goalPositionHold.calculate(GOAL_HEIGHT, z_hat_);  //go up to the goal
                    // upCom = GOAL_UP_VELOCITY;
                    forwardCom = GOAL_FORWARD_SEARCH;
                }

                if (detected_target.size() > 0) {
                    RCLCPP_WARN(this->get_logger(), "DETECTED TARGET - APPROACHING GOAL!");

                    auto_state_ = approachGoal;
                }
                    
                break;
            } case approachGoal: {
                if (detected_target.size() > 0) {
                    yawCom = xPID_.calculate(GOAL_X_OFFSET, tx, dt);
                    upCom = -yPID_.calculate(GOAL_APPROACH_ANGLE, ty, dt);
                    forwardCom = GOAL_CLOSURE_COM;

                    if ((tz < GOAL_DISTANCE_TRIGGER && goalColor == orange) || (tz < GOAL_DISTANCE_TRIGGER && goalColor == yellow)) {
                        scoreTimeStart = millis();
                        auto_state_ = scoringStart;
                    }
                } else {
                    auto_state_ = goalSearch;
                    goalYawDirection = searchDirection();  //randomize search direction
                    ballGrabber.closeGrabber(control_mode_);
                }
                break;
            } case scoringStart: {
                //after correction, we can do goal alignment with a yaw and a translation 
                yawCom = SCORING_YAW_COM;
                forwardCom = SCORING_FORWARD_COM;
                upCom = -SCORING_UP_COM;

                if (scoreTimeStart < millis() - scoreTime) {
                    auto_state_ = shooting;     
                    shootingTimeStart = millis();
                    break;
                }
                break;
            } case shooting: {
                yawCom = 0;
                forwardCom = SHOOTING_FORWARD_COM;
                upCom = -SHOOTING_UP_COM;

                ballGrabber.shoot(control_mode_);
                catches_ = 0;

                if (shootingTimeStart < millis() - shootingTime) {
                    ballGrabber.closeGrabber(control_mode_);
                    scoredTimeStart = millis();
                    auto_state_ = scored;
                    break;
                }
                break;
            } case scored: {
                // ballGrabber.closeGrabber(control_mode_);

                yawCom = 0;
                forwardCom = SCORED_FORWARD_COM;
                upCom = SCORED_UP_COM;

                if (scoredTimeStart < millis() - scoredTime) {
                    auto_state_ = searching;
                    // searching timer
                    searchingTimeStart = millis();
                    searchYawDirection = searchDirection();  //randomize the search direction
                    break;
                }
                break;
            } default: {
                //shouldn't get here
                yawCom = 0;
                forwardCom = 0;
                upCom = 0;
                break;
            }

            if (detected_target.size() > 0) {
                std::cout << detected_target.size() << " targets detected: ";
                for (int i = 0; i < detected_target.size(); i++) {
                    std::cout << detected_target[i] << ", ";
                }
                std::cout << std::endl;
            }
        } //End auto_mode switch
    } else {
        //Blimp is lost
        forwardCom = 0.0;
        upCom = 0.0;
        yawCom = 0.0;
    }

    //publish autonomous state machine info to Basestation
    state_machine_msg.data = auto_state_;

    state_machine_publisher->publish(state_machine_msg);
    //safty height 
    // if (z_hat_ > MAX_HEIGHT) {
    //     upCom = -1;
    // }
    //translation velocity and command
    // Serial.print(">z v:");
    // Serial.println(-yekf.v);
    // Serial.print(">z com:");
    // Serial.println(translationCom);

    //PID controllers
    float yawMotor = 0.0;
    float upMotor = 0.0;
    float forwardMotor = 0.0;
    float translationMotor = 0.0;

    //hyperbolic tan for yaw "filtering"
    float deadband = 2.0; // deadband for filteration
    debug_msg.data[2] = yawCom;
    yawMotor = yawPID_.calculate(yawCom, yawRateFilter.last, dt);  
    debug_msg.data[3] = yawMotor;
    if (abs(yawCom-yawRateFilter.last) < deadband) {
        yawMotor = 0;
    } else {
        yawMotor = tanh(yawMotor)*abs(yawMotor);

    }
    // debug_publisher->publish(debug_msg);
    //TO DO: improve velocity control
    // upMotor = zPID___.calculate(upCom, kf.v, dt); //up velocity from barometer
    // What's up motor? :)
    upMotor = upCom;

    if (USE_EST_VELOCITY_IN_MANUAL) {
        //using kalman filters for the current velosity feedback for full-blimp_state feeback PID controllers

        // forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
        // float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
        // translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
        // float translationMotor = translationPID.calculate(translationCom, kal_vel.y_vel_est, dt); 
    } else {
        //without PID
        forwardMotor = forwardCom;
        translationMotor = translationCom;
    }

    //motor debug
    // debug_msg.data.data[0] = yawCom;
    // debug_msg.data.data[1] = upCom;
    // debug_msg.data.data[2] = translationCom;
    // debug_msg.data.data[3] = forwardCom;
    // debug_msg.data.size = 4;

    // debug_msg.data[4] = yawCom;

    // if (abs(floor(upMotor)) > abs(floor(yawMotor))){
    //     yawCom = 0;
    // } else{
    //     upMotor = 0;
    // }
    
    debug_msg.data[4] = yawMotor;
    debug_msg.data[5] = upMotor;
    debug_msg.data[6] = translationMotor;
    debug_msg.data[7] = forwardMotor;
    

    // debug_msg.data.data[0] = forward_msg;
    // debug_msg.data.data[1] = yaw_msg;
    // debug_msg.data.data[2] = up_msg;
    // debug_msg.data.data[3] = translation_msg;
    // debug_msg.data.size = 4;

    //test target messages
    // debug_msg.data.data[0] = targets[0];
    // debug_msg.data.data[1] = targets[1];
    // debug_msg.data.data[2] = targets[2];
    // debug_msg.data.data[3] = targets[3];
    // debug_msg.data.data[4] = targets[4];
    // debug_msg.data.data[5] = targets[5];
    // debug_msg.data.data[6] = targets[6];
    // debug_msg.data.data[7] = targets[7];
    // debug_msg.data.data[8] = targets[8];
    // debug_msg.data.size = 9;

    

    // //Serial.print(">up current:");
    // //Serial.println(kf.v);
    // //float yawCurrent =  (float)yawRateFilter.last;
    // //Serial.print(">yaw current");
    // //Serial.println(yawCurrent);
    // //Serial.print(">forward current:");
    // //Serial.println(xekf.v);
    // //Serial.print(">translation current:");
    // //Serial.println(yekf.v);

    //gimbal + motor updates
    ballGrabber.update();

    // neeed to verify this
    if (loop_time < 10 + firstMessageTime) {
        // publish_log("Im in state_machine_callback dt<10+firstMessage");
        // publish_log(std::to_string(loop_time));
        // publish_log("Im in state_machine_callback dt<10+firstMessage/firstMessage");
        // publish_log(std::to_string(firstMessageTime));
        // rollOffset.filter(BerryIMU.gyr_rateXraw);

        //zero motors while filters converge and esc arms
        motorControl.update(0, 0, 0, 0, 0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
    } else {
        if (control_mode_ == manual && !MOTORS_OFF) {
            // publish_log("Im in state_machine_callback dt<10+firstMessage/manual");
            //forward, translation, up, yaw, roll
            if (!ZERO_MODE) motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
            // debug_msg.data = {motorControl.upLeft, motorControl.forwardLeft, motorControl.upRight, motorControl.forwardRight};
            // debug_publisher->publish(debug_msg);
            debug_msg.data[10] = motorControl.upLeft;
            // debug_msg.data[10] = 9999;
            debug_msg.data[11] = motorControl.forwardLeft;
            debug_msg.data[12] = motorControl.upRight;
            debug_msg.data[13] = motorControl.forwardRight;
            debug_publisher->publish(debug_msg);

            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upLeft, motorControl.forwardLeft);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upRight, motorControl.forwardRight);
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);

            // RCLCPP_INFO(this->get_logger(), "Left: %.2f (%.2f us), Right: %.2f (%.2f us)", leftGimbal.getServoAngle(), leftGimbal.getServoUS(), rightGimbal.getServoAngle(), rightGimbal.getServoUS());

        } else if (control_mode_ == autonomous && !MOTORS_OFF) {
            debug_msg.data[10] = motorControl.upLeft;
            debug_msg.data[11] = motorControl.forwardLeft;
            debug_msg.data[12] = motorControl.upRight;
            debug_msg.data[13] = motorControl.forwardRight;
            motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        } else if(MOTORS_OFF){
            motorControl.update(0,0,0,0,0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        } else {
            motorControl.update(0,0,0,0,0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        }
    }

    // float endTime = micros();
    // float netTime = (endTime - startTime)/MICROS_TO_SEC;
    // publish_log("State Machine Call Back Time:");
    // publish_log(std::to_string(netTime));
    debug_publisher->publish(debug_msg);

    if (auto_state_ != last_state_) {

        std::string state_str;
        switch(auto_state_) {
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
        
        RCLCPP_INFO(this->get_logger(), "State changed to %s", state_str.c_str());
    }
    last_state_ = auto_state_;
}

void CatchingBlimp::publish_log(std::string message) const {
    auto log_msg = std_msgs::msg::String();
    log_msg.data = message;
    log_publisher->publish(log_msg);
    // RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

void CatchingBlimp::auto_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
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

void CatchingBlimp::calibrateBarometer_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
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

void CatchingBlimp::baro_subscription_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: %.4f", msg->data);

    base_baro_ = msg->data;

    if (!baro_init_) {
        RCLCPP_INFO(this->get_logger(), "Base barometer initialized.");
        baro_init_ = true;
    }

    // //filter base station data
    // baroOffset.filter(baseBaro - BerryIMU.comp_press);

    //heartbeat
    //update last message time
    lastMsgTime = micros()/MICROS_TO_SEC;

    //If teensy comes out of lost control mode, put it in manual control mode
    if (control_mode_ == lost) {
        control_mode_ = manual;
    }
}

void CatchingBlimp::grab_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (grabCom == 0 && msg->data) {
        grabCom = 1;
        publish_log("Going for a catch...");
    } else if (grabCom == 1 && !msg->data) {
        grabCom = 0;
        publish_log("Hopefully I got a balloon!");
    }
}

void CatchingBlimp::kill_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (msg->data == true) {
        publish_log("I'm ded xD");
        motorControl.update(0,0,0,0,0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
    }
}

void CatchingBlimp::shoot_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    if (shootCom == 0 && msg->data) {
        shootCom = 1;
        publish_log("I'm shooting my shot...");
    } else if (shootCom == 1 && !msg->data) {
        shootCom = 0;
        publish_log("What a shot!");
    }
}

void CatchingBlimp::motor_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    //commands from basestation
    // forward_msg = msg->data.data[3];
    // up_msg = msg->data.data[1];
    // yaw_msg = msg->data.data[0];
    // translation_msg = msg->data.data[2];

    forward_msg = msg->data[3];
    up_msg = msg->data[1];
    yaw_msg = msg->data[0];
    translation_msg = msg->data[2];

    // debug_msg.data[5] = forward_msg
    // debug_msg.data[6] = up_msg
    // debug_msg.data[7] = translation_msg
    // motorControl.yawLeft = yaw_msg;
    // motorControl.upLeft = up_msg;
    // motorControl.forwardLeft = forward_msg;

    // auto debug_msg = std_msgs::msg::Float64MultiArray();
    // debug_msg.data = {motorControl.yawLeft, motorControl.upLeft, translation_msg, motorControl.forwardLeft};
    // debug_publisher->publish(debug_msg);

    // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
    // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
    // leftGimbal.updateGimbal(leftReady && rightReady);
    // rightGimbal.updateGimbal(leftReady && rightReady);

    // char motorCommands[100];  // Size depending on the expected maximum length of your combined string
    // sprintf(motorCommands, "Teensy Motor Commands\nYaw: %.2f\nUp: %.2f\nTranslation: %.2f\nForward: %.2f\n", yaw_msg, up_msg, translation_msg, forward_msg);
    // publish_log(motorCommands);
}

void CatchingBlimp::goal_color_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
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

void CatchingBlimp::avoidance_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        // avoidance[i] = msg->data.data[i];
        avoidance[i] = msg->data[i];
    }
}

// void targets_subscription_callback(const geometry_msgs::msg::Point & msg) const
// {
//     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

//     // object of interest with xyz (3 elements in total)
//     // for (size_t i = 0; i < 3; ++i) {
//     //     // targets[i] = msg->data.data[i];
//     //     targets[i] = msg->data[i];
//     // }

//     targets[0] = msg->x;

//     targets[1] = msg->y;

//     targets[2] = msg->z;

// }

void CatchingBlimp::targets_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    // object of interest with xyz (3 elements in total)
    for (size_t i = 0; i < 3; ++i) {
        // targets[i] = msg->data.data[i];
        targets_[i] = msg->data[i];
    }

    // targets[0] = msg->x;

    // targets[1] = msg->y;

    // targets[2] = msg->z;

}

void CatchingBlimp::pixels_subscription_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
    auto pixels_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        //pixels[i] = pixels_msg->data.data[i];
        pixels[i] = pixels_msg->data[i];
    }
}

// // //pulse function for adding ultrasonic
// void Pulse() {
//     if (digitalRead(interruptPin) == HIGH) {
//         // start measuring
//         pulseInTimeBegin = micros();
//     }
//     else {
//         // stop measuring
//         pulseInTimeEnd = micros();
//         newPulseDurationAvailable = true;
//     }
// }

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
    
    double x_p, x_i, x_d, y_p, y_i, y_d, z_p, z_i, z_d, yaw_p, yaw_i, yaw_d;
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
        this->get_parameter("yaw_d", yaw_d) 
    ){
        //Set gains
        xPID_ = PID(x_p, x_i, x_d);  // left and right
        yPID_ = PID(y_p, y_i, y_d);  // up and down
        zPID_ = PID(z_p, z_i, z_d);  // unused
        yawPID_ = PID(yaw_p, yaw_i, yaw_d); // yaw correction 

        RCLCPP_INFO(this->get_logger(), "Yaw PIDs: p=%.2f", yaw_p);

        return true;
    } else {
        return false;
    }
}