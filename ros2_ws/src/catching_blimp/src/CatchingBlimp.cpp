#include "CatchingBlimp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//blimp game parameters
int blimpColor = BLIMP_COLOR;
int goalColor = GOAL_COLOR;

//msg for commands
float forward_msg = 0;
float yaw_msg = 0;
float up_msg = 0;

//timers for state machine
bool backingUp = false;

//grabber data
int shoot = 0;
int grab = 0;
int shootCom = 0;
int grabCom = 0;

//avoidence data
int quadrant = 10;

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

// EMAFilter areaFilter(0.5);

//baro offset computation from base station value
// EMAFilter baroOffset(0.5);

//roll offset computation from imu
// EMAFilter rollOffset(0.5);

//ball grabber object
TripleBallGrabber ballGrabber;

CatchingBlimp::CatchingBlimp() : 
    Node("catching_blimp_node"), 
    count_(0), 
    imu_init_(false), 
    baro_init_(false), 
    baro_calibration_offset_(0.0),
    baro_sum_(0.0), 
    baro_count_(0),
    z_hat_(0), 
    catches_(0), 
    control_mode_(INITIAL_MODE), 
    auto_state_(searching),
    rollrate_command_(0),
    yawrate_command_(0),
    roll_update_count_(0),
    target_detected_(false),
    target_type_(no_target),
    target_id_(-1) {

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
    z_est_.initialize();
    z_lowpass_.setAlpha(0.1);

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

    heartbeat_msg_.data = true;

    //Initialize state message
    state_msg_.data.reserve(2);
    state_msg_.data.push_back(0);
    state_msg_.data.push_back(0);
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
    rollRateFilter.filter(BerryIMU.gyr_rateXraw);

    std::vector<double> euler_angles = madgwick.get_euler();
    double roll = euler_angles[0];

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

    if (!baro_init_) return;

    cal_baro_ = 44330 * (1 - pow(((BerryIMU.comp_press - baro_calibration_offset_)/base_baro_), (1/5.255)));
    baro_sum_ += cal_baro_;
    baro_count_++;

    //Average barometer every 5 samples (5Hz)
    if (baro_count_ == 5) {
        double baro_mean_ = baro_sum_/(double)baro_count_;
        
        z_est_.partialUpdate(baro_mean_);

        //Lowpass current estimate
        z_hat_ = z_lowpass_.filter(z_est_.xHat(0));

        baro_sum_ = 0.0;
        baro_count_ = 0;
    }
}

void CatchingBlimp::state_machine_callback() {

    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - state_machine_time_).seconds();
    state_machine_time_ = now;

    auto debug_msg = std_msgs::msg::Float64MultiArray();

    //object avoidence commands to overide search and computer vision
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
    searchYawDirection = searchDirection(); // this used to only be in manual, moved it here so its also reflected in autonomy -vd 11/20
    if (control_mode_ == manual) {
        // publish_log("Im in state_machine_callback, manual");
        //get manual data
        //all motor commands are between -1 and 1
        //set max yaw command to 120 deg/s

        yawrate_command_ = -yaw_msg*120;

        if (USE_EST_VELOCITY_IN_MANUAL) {
            //set max velocities 2 m/s
            up_command_ = up_msg*2.0;
            forward_command_ = forward_msg*2.0;
        } else {
            //normal mapping using max esc command 
            up_command_ = up_msg*750.0; //up is negative
            forward_command_ = forward_msg*750.0;
        }

        //check if shooting should be engaged
        //this block switches the control mode to the oposite that it is currently in
        if (shoot != shootCom) {
            shoot = shootCom;

            //change shoot control mode
            if (ballGrabber.state_ == 2) {
                //stop shooting
                ballGrabber.closeGrabber(control_mode_);
            } else {
                //reset catch counter
                catches_ = 0;

                //go back to searching
                // auto_state_ = searching;
                // searching timer
                search_start_time_ = now;
                searchYawDirection = searchDirection();  //randomize the search direction

                //start shooting
                ballGrabber.shoot(control_mode_);
            }
        //check if grabbing should be engaged
        //this block switches the control mode to the oposite that it is currently in
        } else if (grab != grabCom) {
            grab = grabCom;

            //change grab control mode
            if (ballGrabber.state_ == 0) {
                ballGrabber.openGrabber(control_mode_);
            } else {
                ballGrabber.closeGrabber(control_mode_);

                //increase catch counter
                catches_++;

                //start catch timmer
                if (catches_ >= 1) {
                    last_catch_time_ = now;
                }
            }
        }
    } else if (control_mode_ == autonomous) {

    //-------------------------Target is determined from the state we are in-------------------------------------------\
    //-------------------------------------------------------------------------------------------------------------
       
        // Modes for autonomous behavior
        switch (auto_state_) {
            // Autonomous state machine
            case searching: {

                //check if goal scoring should be attempted
                if (catches_ >= 1 && ((now - last_catch_time_).seconds() >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches_-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
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

                //begin search pattern spinning around at different heights
                if (!target_detected_) {

                    //keep ball grabber closed
                    ballGrabber.closeGrabber(control_mode_);

                    //use object avoidence
                    double avoidanceMinVal = 1000.0; // Initialize 
                    int avoidanceMinIndex = 10;

                    // Iterate through the vector to find the minimum value and its index
                    // find the minimum distance and its corresponding quadrant number (1-9)
                    // TODO: IMPLEMENT AND TEST AVOIDANCE
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
                        yawrate_command_ = yawA;
                        forward_command_ = forwardA;
                        up_command_ = upA;

                    } else {
                        //search behavior (no detected_target)
                        //spin in a small circle looking for a game ball
                        //randomize the search direciton

                        // yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        // up_command_ = 0;    //is overriden later, defined here as a safety net
                        // forward_command_ = GAME_BALL_FORWARD_SEARCH;
                        
                        // Timeline:
                        // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
                        double elapsedSearchingTime = (now - search_start_time_).seconds();
                        std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";

                        if (elapsedSearchingTime < TIME_TO_SEARCH) {
                            backingUp = false;
                            yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            forward_command_ = GAME_BALL_FORWARD_SEARCH;

                            if (z_hat_ > CEIL_HEIGHT) {
                                up_command_ = GAME_BALL_VERTICAL_SEARCH; //down
                            }else if (z_hat_ < FLOOR_HEIGHT) {
                                up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
                            }else{
                                up_command_ = 150;
                            }
                        } else if (elapsedSearchingTime < TIME_TO_SEARCH + TIME_TO_BACKUP) {
                            if (!backingUp) {
                                backingUp = true;
                                searchYawDirection = searchDirection();
                            }   
                            message += " Backup!";
                            
                            yawrate_command_ = 35*searchYawDirection;
                            forward_command_ = -240;
                            up_command_ = 100;
                        } else {
                            message += " Reset!";
                            search_start_time_ = now;
                        }

                        yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        forward_command_ = GAME_BALL_FORWARD_SEARCH;

                        if (z_hat_ > CEIL_HEIGHT) {
                            up_command_ = -GAME_BALL_VERTICAL_SEARCH; //down
                        } else if (z_hat_ < FLOOR_HEIGHT) {
                            up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
                        } else {
                            up_command_ = 150; //up
                        }

                        // Prolly hit a net
                        // if (20000 < millis()- searchingTimeStart){
                        //     double backupTimer = millis();

                        //     std::string message = "20+ seconds.";

                        //     if (2200 > millis() - backupTimer) {
                        //         message += " first 2 seconds.";
                        //         searchYawDirection = searchDirection();
                        //         yawrate_command_ = 60*searchYawDirection;
                        //         up_command_ = 50;    //is overriden later, defined here as a safety net
                        //         forward_command_ = -200;
                        //     }

                        //     searchingTimeStart = millis(); 
                        //     publish_log(message.c_str());
                        // }

                        //move up and down within the set boundry
                        // if (z_hat_ > CEIL_HEIGHT) {
                        //     // if (wasUp) wasUp = false;
                        //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     up_command_ = GAME_BALL_VERTICAL_SEARCH; //down
                        //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
                        // }

                        // if (z_hat_ < FLOOR_HEIGHT) {
                        //     // if (!wasUp) wasUp = true;
                        //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
                        //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
                        // }

                        // if (z_hat_ <= CEIL_HEIGHT && z_hat_ >=FLOOR_HEIGHT) {
                        //     // if (wasUp) wasUp = false;
                        //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                        //     up_command_ = 0;
                        //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
                        // }
                    }
                } else {
                    //move to approaching game ball
                    RCLCPP_INFO(this->get_logger(), "Switched from Search to Approach");
                    auto_state_ = approach;

                    //start approaching timer
                    approach_start_time_ = now;
                }
                break;
            } case approach: {
                // max time to approach
                if ((now - approach_start_time_).seconds() >= MAX_APPROACH_TIME) {
                    auto_state_ = searching;
                    // searching timer
                    search_start_time_ = now;
                }
   
                //check if target is still valid
                if (target_detected_) {
                    //seeing a target
                    //move toward the balloon
                    yawrate_command_ = xPID_.calculate(GAME_BALL_X_OFFSET, target_.x, dt);
                    up_command_ = yPID_.calculate(GAME_BALL_Y_OFFSET, target_.y, dt);  

                    forward_command_ = GAME_BALL_CLOSURE_COM;

                    //check if the gate should be opened
                    if (target_.z < BALL_GATE_OPEN_TRIGGER) {
                        ballGrabber.openGrabber(control_mode_);

                        //check if the catching mode should be triggered
                        if (target_.z < BALL_CATCH_TRIGGER) {
                            auto_state_ = catching;

                            //start catching timer
                            catch_start_time_ = now;
                        }
                    }
                    //if target is lost within 1 second
                    //remember the previous info about where the ball is 
                } else {
                    // Target memory timeout - close grabber and start searching again
                    ballGrabber.closeGrabber(control_mode_);
                    auto_state_ = searching;

                    // Reset searching timer and direction
                    search_start_time_ = now;
                    ballGrabber.closeGrabber(control_mode_);
                    searchYawDirection = searchDirection();  //randomize the search direction
                }

                break;
            } case catching: {

                //Go slower when we get up close
                if (target_.z > 5.0) {
                    forward_command_ = CATCHING_FORWARD_COM;
                    up_command_ = CATCHING_UP_COM;
                } else {
                    forward_command_ = 200;
                    up_command_ = CATCHING_UP_COM;
                }

                yawrate_command_ = 0;

                //Turn on the SUCK
                if (ballGrabber.is_open()) {
                    ballGrabber.suck();
                }

                if ((now - catch_start_time_).seconds() >= TIME_TO_CATCH) {
                    //catching ended, start caught timer
                    auto_state_ = caught;
                    caught_start_time_ = now;

                    ballGrabber.closeGrabber(control_mode_);

                    //increment number of catches_
                    catches_ = catches_ + 1;

                    //start catch timmer
                    last_catch_time_ = now;
                }

                break;
            } case caught: {
                if (catches_ > 0) {
                    //if a target is seen right after the catch
                    if (target_detected_ && catches_ < TOTAL_ATTEMPTS) {
                        //approach next game ball if visible
                        auto_state_ = searching;

                        // searching timer
                        search_start_time_ = now;
                        searchYawDirection = searchDirection();  //randomize the search direction
                    }

                    //decide if the blimp is going to game ball search or goal search
                    if ((now - caught_start_time_).seconds() >= TIME_TO_CAUGHT) {
                        if (catches_ >= TOTAL_ATTEMPTS) {
                            auto_state_ = goalSearch;
                            goalYawDirection = searchDirection();  //randomize search direction
                        } else {
                            auto_state_ = searching;
                            // searching timer
                            search_start_time_ = now;
                            searchYawDirection = searchDirection();  //randomize the search direction
                        }
                    }

                    forward_command_ = CAUGHT_FORWARD_COM;
                    up_command_ = CAUGHT_UP_COM;
                    yawrate_command_ = 0;
                } else {
                    auto_state_ = searching;

                    // searching timer
                    search_start_time_ = now;
                    searchYawDirection = searchDirection();  //randomize the search direction
                }
                break;
            } case goalSearch: {

                //keep ball grabber closed
                ballGrabber.closeGrabber(control_mode_);

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
                    yawrate_command_ = yawA;
                    forward_command_ = forwardA;
                    up_command_ = upA;
                } else {
                    //goal search behavior
                    //randomize the diretion selection
                    yawrate_command_ = GOAL_YAW_SEARCH*goalYawDirection;
                    up_command_ = goalPositionHold.calculate(GOAL_HEIGHT, z_hat_);  //go up to the goal
                    forward_command_ = GOAL_FORWARD_SEARCH;
                }

                if (target_detected_) {
                    RCLCPP_WARN(this->get_logger(), "DETECTED GOAL - APPROACHING!");
                    goal_approach_start_time_ = now;
                    auto_state_ = approachGoal;
                }

                break;

            } case approachGoal: {

                if (target_detected_) {
                    yawrate_command_ = xPID_.calculate(GOAL_X_OFFSET, target_.x, dt);
                    up_command_ = yPID_.calculate(GOAL_Y_OFFSET, target_.y, dt);

                    if (target_.z > 4.0) {
                        forward_command_ = GOAL_CLOSURE_COM;
                    } else {
                        forward_command_ = GOAL_CLOSE_COM;
                    }

                    if (target_.z < GOAL_DISTANCE_TRIGGER) {
                        score_start_time_ = now;
                        auto_state_ = scoringStart;
                    }
                } else {
                    // Target memory timeout - start searching for goals again
                    auto_state_ = goalSearch;
                    goalYawDirection = searchDirection();  //randomize search direction
                }

                break;

            } case scoringStart: {
                //after correction, we can do goal alignment with a yaw and a translation 
                yawrate_command_ = SCORING_YAW_COM;
                forward_command_ = SCORING_FORWARD_COM;
                up_command_ = SCORING_UP_COM;

                if ((now - score_start_time_).seconds() >= TIME_TO_SCORE) {
                    auto_state_ = shooting;
                    shoot_start_time_ = now;
                    break;
                }
                break;

            } case shooting: {
                yawrate_command_ = 0;
                forward_command_ = SHOOTING_FORWARD_COM;
                up_command_ = SHOOTING_UP_COM;

                ballGrabber.shoot(control_mode_);

                if ((now - shoot_start_time_).seconds() >= TIME_TO_SHOOT) {
                    ballGrabber.closeGrabber(control_mode_);
                    score_start_time_ = now;
                    auto_state_ = scored;
                    break;
                }
                break;

            } case scored: {
                ballGrabber.closeGrabber(control_mode_);

                yawrate_command_ = 0;
                forward_command_ = SCORED_FORWARD_COM;
                up_command_ = SCORED_UP_COM;

                if ((now-score_start_time_).seconds() >= TIME_TO_SCORED) {
                    //Reset catch counter and return to ball search state
                    catches_ = 0;
                    auto_state_ = searching;
                    search_start_time_ = now;
                    searchYawDirection = searchDirection();  //randomize the search direction
                    break;
                }
                break;
            } default: {
                //shouldn't get here
                yawrate_command_ = 0;
                forward_command_ = 0;
                up_command_ = 0;
                break;
            }
        } //End auto_mode switch
    } else {
        //Blimp is lost
        forward_command_ = 0.0;
        up_command_ = 0.0;
        yawrate_command_ = 0.0;
    }
    
    up_motor_ = up_command_; //up for motor
    forward_motor_ = forward_command_; //forward for motor

    //gimbal + motor updates
    ballGrabber.update();

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
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    rclcpp::Time now = this->get_clock()->now();

    // Todo: Make sure object type matches the current autonomous state!

    // Make sure target was detected by checking for a -1 in the first element
    if (msg->data[0] >= 0) {

        // Target in sight - update filters if not previously seen
        if (!target_detected_) {
            // New target detected!
            target_detected_ = true;

            // Reset XYZ detection filters so value is set to current target
            xFilter.reset();
            yFilter.reset();
            zFilter.reset();
        }
        
        if ((int)msg->data[3] != target_id_) {
            // Target changed!
            // Also reset filter if the target changes
            target_id_ = (int)msg->data[3];

            xFilter.reset();
            yFilter.reset();
            zFilter.reset();
        }
        
        if ((target_type)msg->data[4] != target_type_) {
            // Target type changed!
            target_type_ = (target_type)msg->data[4];

            xFilter.reset();
            yFilter.reset();
            zFilter.reset();
        }

        // Center target at (0,0)
        geometry_msgs::msg::Point centered_target;
        centered_target.x = msg->data[0] - 320;
        centered_target.y = msg->data[1] - 240;
        centered_target.z = msg->data[2];

        target_.x = xFilter.filter(centered_target.x);
        target_.y = yFilter.filter(centered_target.y);
        target_.z = zFilter.filter(centered_target.z);

        // Update target memory every time when a detection takes place
        target_memory_time_ = now;

    } else {
        // No target in sight
        if (target_detected_) {
            // Pursue "imaginary" target for a bit in case it reappears in frame
            if ((now - target_memory_time_).seconds() < TARGET_MEMORY_TIMEOUT) {
                // Set filters to automatically converge to desired

                if (target_type_ == ball) {
                    // Missing target was a game ball
                    target_.x = xFilter.filter(GAME_BALL_X_OFFSET);
                    target_.y = yFilter.filter(GAME_BALL_Y_OFFSET);
                    target_.z = zFilter.filter(BALL_CATCH_TRIGGER);
                } else {
                    // Missing target was a goal
                    target_.x = xFilter.filter(GOAL_X_OFFSET);
                    target_.y = yFilter.filter(GOAL_Y_OFFSET);
                    target_.z = zFilter.filter(GOAL_DISTANCE_TRIGGER);
                }
            } else {
                // Target lost
                target_detected_ = false;
                target_id_ = -1;
                target_type_ = no_target;
            }
        }
    }

    if (VISION_PRINT_DEBUG) {
        if (target_detected_) {
            std::string target_str = (target_type_ == ball ? "Ball" : (target_type_ == goal ? "Goal" : "None"));
            std::string detection_str = std::string("Target: ").append(target_str).append(" ID ").append(std::to_string(target_id_)).append(")");
            detection_str.append(" at (" + std::to_string(target_.x) + ", " + std::to_string(target_.y) + ", " + std::to_string(target_.z));

            RCLCPP_INFO(this->get_logger(), detection_str.c_str());
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