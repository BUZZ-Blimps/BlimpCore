#ifndef CATCHING_BLIMP_HPP
#define CATCHING_BLIMP_HPP

//C includes
#include <stdio.h>
#include <stdlib.h>

//C++ includes
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <vector>
#include <string>
#include <numeric>
#include <deque>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//Message type includes
#include <std_msgs/msg/string.hpp> //include the message type that needs to be published (teensy data)
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

// #include "MotorControl.hpp"
#include "OPI_IMU.hpp"
#include "Madgwick_Filter.hpp"
#include "PID.hpp"
#include "EMAFilter.hpp"
#include "BangBang.hpp"
#include "tripleBallGrabber.hpp"
// #include "Gimbal.hpp"
#include "MotorControl_V2.hpp"
#include "ZEstimator.hpp"
#include "TOF_Sense.hpp"
#include "math_helpers.hpp"

#include <wiringPi.h>

// blimp mode
#define BLIMP_COLOR               red      //either red or blue
#define GOAL_COLOR                orange    //either orange or yellow

// debug mode
#define INITIAL_MODE              manual
#define GIMBAL_DEBUG              false

// Motor debugging
#define MOTOR_PRINT_DEBUG         false
#define ZERO_MODE                 false
#define VERT_MODE                 false
#define YAW_MODE                  false

// Vision debugging
#define VISION_PRINT_DEBUG        true

#define USE_DISTANCE_IN_BALL_APPROACH   false
#define BASKET_CAMERA_VERTICAL_OFFSET   -0.3     // m vertical distance between center of catching basket and camera
#define BALL_TRACKING_TESTING           false

//optional controllers
#define USE_EST_VELOCITY_IN_MANUAL  false    //use false to turn off the velosity control to see the blimp's behavior 
#define USE_OBJECT_AVOIDENCE        false     //use false to turn off the obstacle avoidance 

//catch search time after one
#define MAX_SEARCH_WAIT_AFTER_ONE     80.0    //max searching 
#define GAME_BALL_WAIT_TIME_PENALTY   0       //should be set to 20, every catch assumed to be 20 seconds long  

//number of catches attempted
#define TOTAL_ATTEMPTS            1    // attempts at catching 
#define MAX_ATTEMPTS              5    // should be set to 5

//flight area parameters
#define CEIL_HEIGHT               3   //m
#define FLOOR_HEIGHT              -1  //m

#define MAX_HEIGHT                12   //m  (unused)
#define GOAL_HEIGHT               1  //m
#define GOAL_HEIGHT_DEADBAND      0.3  //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER    2.5   // m distance for blimp to trigger goal score 	
#define FAR_APPROACH_THRESHOLD   3.0   // m distance for blimp to alignment submode switching in approach state
#define BALL_GATE_OPEN_TRIGGER   3.5   // m distance for blimp to open the gate 	
#define BALL_CATCH_TRIGGER       1.5   // m distance for blimp to start the open-loop control
#define AVOID_TRIGGER            0.8   // m distance for blimp to start the open-loop control

//object avoidence motor coms
#define FORWARD_AVOID             125  // 25% throttle
#define YAW_AVOID                 30   // deg/s
#define UP_AVOID                  125  // % throttle 

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -20  // deg/s
#define GAME_BALL_FORWARD_SEARCH  200  // 30% throttle 
#define GAME_BALL_VERTICAL_SEARCH 200  // 45% throttle

#define GAME_BALL_CLOSURE_COM     250  //approaching at 20% throttle cap
#define GAME_BALL_X_OFFSET        20    //offset magic number (more to the left)
#define GAME_BALL_Y_OFFSET        160   //approach magic number

#define GOAL_CLOSURE_COM          275  //forward command 25% throttle
#define GOAL_CLOSE_COM            180 
#define GOAL_X_OFFSET             70    //more to the left
#define GOAL_Y_OFFSET             190   //height alignment (approach down)

#define CATCHING_FORWARD_COM      430  //catching at 50% throttle 
#define CATCHING_UP_COM           50   //damp out pitch

#define TIME_TO_SEARCH            15.0
#define TIME_TO_BACKUP            5.0
#define TIME_TO_CATCH             4.2 //seconds
#define TIME_TO_CAUGHT            2.5
#define TIME_TO_SCORE             1.7
#define TIME_TO_SHOOT             3.5
#define TIME_TO_SCORED            3.5
#define MAX_APPROACH_TIME         15.0
// #define MAX_APPROACH_TIME         600.0
#define ALIGNMENT_DURATION        0.0  // seconds to wait between far approach and near approach
#define TARGET_PREDICTION_USE_DELAY     0.5 // seconds to wait until prediction is used
#define TARGET_MEMORY_TIMEOUT     2.0  // seconds to wait until ID/detection is moved on from
#define ALIGN_PREDICT_HORIZON     0.0  // seconds to forward predict game ball position for alignment

#define CAUGHT_FORWARD_COM        250  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             40

#define GOAL_YAW_SEARCH           15
#define GOAL_FORWARD_SEARCH       200  //200 40% throttle
#define GOAL_UP_VELOCITY          250

//goal alignment test
#define ALIGNING_YAW_COM           10   //test
#define ALIGNING_FORWARD_COM       180  //test
#define ALIGNING_UP_COM            100  //test
#define ALIGNING_TRANSLATION_COM   300  //test

#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       350 //80% throttle
#define SCORING_UP_COM            180

#define SHOOTING_FORWARD_COM      400  //counter back motion 
#define SHOOTING_UP_COM           200
//counter moment (right now we do want to shoot up because ball sinks)

#define SCORED_FORWARD_COM        -400
#define SCORED_UP_COM             0

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ     100.0
#define BARO_LOOP_FREQ            50.0
#define STATE_MACHINE_FREQ        30.0
#define OPTICAL_LOOP_FREQ         55.0

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0
#define GYRO_Y_CONSTANT           323.45

#define BALL_APPROACH_THRESHOLD   2500
#define BALL_CATCH_THRESHOLD      62000

//OrangePi5 Pinout
// #define L_Pitch                   5     // was 2
// #define L_Yaw                     3     //not used, not changed
// #define R_Pitch                   0
// #define R_Yaw                     9     //not used, was 5

// Todo: determine these pin numbers after re-wiring
#define GATE_S                    5
#define PIN_SCORING               0

//10 is a valid brushless motor
#define PIN_LEFT_UP               16
#define PIN_LEFT_FORWARD          2
#define PIN_RIGHT_UP              8
#define PIN_RIGHT_FORWARD         10

//***********************************************//

// Constants
#define MICROS_TO_SEC             1000000.0
#define BUFFER_LEN                512
#define MAX_TARGETS               100
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000

enum autoState {
    searching,
    approach,
    catching,
    caught,
    goalSearch,
    approachGoal,  //To Do: add goal aligntment (PID) in if, or add another state "align"
    scoringStart,
    shooting,
    scored,
    no_state
};

enum blimpState {
    manual,
    autonomous,
    lost,
};

enum approachState {
    far_approach,
    alignment,
    near_approach
};

enum grabberState {
    opened,
    closed,
};

enum blimpType {
    blue,
    red
};

enum goalType {
    orange,
    yellow
};

enum gameballType {
    green,
    pink
};

enum target_type {
    ball,
    goal,
    no_target
};

struct TargetData {
  double x;
  double y;
  double z;
  double theta_x;
  double theta_y;
  rclcpp::Time timestamp;
  int id;
  target_type type;
};

class CatchingBlimp: public rclcpp::Node {
public:
    CatchingBlimp();
        
private: 
    //Global variables
    //sensor fusion objects
    OPI_IMU BerryIMU;
    Madgwick_Filter madgwick;
    //Z estimator sensor variance
    double R_bar = 1.0;
    double R_lid = 0.1;

    // MotorControl motorControl;
    // Gimbal leftGimbal;
    // Gimbal rightGimbal;
    MotorControl_V2 motorControl_V2;

    // //Goal positioning controller
    // BangBang goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY); //Dead band, velocity to center itself

    // //filter on yaw gyro
    // EMAFilter yawRateFilter(0.2);
    // EMAFilter rollRateFilter(0.5);

    // //Low pass filter for computer vision parameters
    // EMAFilter xFilter(0.5);
    // EMAFilter yFilter(0.5);
    // EMAFilter zFilter(0.5);
    // EMAFilter theta_xFilter(0.5);
    // EMAFilter theta_yFilter(0.5);

    //Goal positioning controller
    BangBang goalPositionHold; //Dead band, velocity to center itself

    //filter on yaw gyro
    EMAFilter yawRateFilter;
    EMAFilter rollRateFilter;

    //Low pass filter for computer vision parameters
    EMAFilter xFilter;
    EMAFilter yFilter;
    EMAFilter zFilter;
    EMAFilter theta_xFilter;
    EMAFilter theta_yFilter;

    // EMAFilter areaFilter(0.5);

    //baro offset computation from base station value
    // EMAFilter baroOffset(0.5);

    //roll offset computation from imu
    // EMAFilter rollOffset(0.5);

    //ball grabber object
    TripleBallGrabber ballGrabber;


    rclcpp::TimerBase::SharedPtr timer_imu;
    rclcpp::TimerBase::SharedPtr timer_baro;
    rclcpp::TimerBase::SharedPtr timer_state_machine;
    rclcpp::TimerBase::SharedPtr timer_heartbeat;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr heading_publisher_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr base_baro_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cal_baro_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabber_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shooter_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_color_subscription;

    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr targets_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr targets_subscription;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr pixels_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr avoidance_subscription;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped blimp_tf_;


    size_t count_;
    std::string blimp_name_;

    std_msgs::msg::Bool heartbeat_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    std_msgs::msg::Float64 z_msg_, z_vel_msg_;
    std_msgs::msg::Int64MultiArray state_msg_;
    std_msgs::msg::Float64MultiArray debug_msg_;

    //blimp game parameters
    int blimpColor = BLIMP_COLOR;
    int goalColor = GOAL_COLOR;

    // Target detection
    bool target_detected_ = false;
    TargetData target_;
    int target_id_;
    target_type target_type_;
    std::deque<TargetData> target_history_;

    //timers for state machine
    bool backingUp = false;

    //grabber data
    int shoot = 0;
    int grab = 0;
    int shootCom = 0;
    int grabCom = 0;

    double searchYawDirection = -1;
    double goalYawDirection = -1;
    

    //Avoidance data
    int quadrant = 10;

    //avoidance data (9 quadrants), targets data and pixel data (balloon, orange goal, yellow goal)
    //1000 means object is not present
    std::vector<double> avoidance = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

    
    bool imu_init_, baro_init_;
    double base_baro_, baro_calibration_offset_, cal_baro_, baro_sum_;
    double lidar_calibration_offset_;
    int baro_count_; 
    double z_hat_;
    double z_hat_2;

    int catches_;

    blimpState control_mode_;
    
    autoState auto_state_;
    autoState last_state_ = no_state;

    approachState approach_state_ = far_approach;
    rclcpp::Time alignment_start_time_;

    //msg for commands
    float forward_msg = 0;
    float yaw_msg = 0;
    float up_msg = 0;

    double forward_motor_, up_motor_, yaw_motor_, roll_rate_motor_;
    double forward_command_, up_command_, yawrate_command_, rollrate_command_;
    double forward_avoidance_, up_avoidance_, yaw_avoidance_;
    int roll_update_count_;
    
    rclcpp::Time start_time_;
    rclcpp::Time state_machine_time_;
    rclcpp::Time target_memory_time_;
    rclcpp::Time last_catch_time_;

    rclcpp::Time search_start_time_;
    rclcpp::Time approach_start_time_;
    rclcpp::Time catch_start_time_;
    rclcpp::Time caught_start_time_;
    rclcpp::Time goal_approach_start_time_;
    rclcpp::Time shoot_start_time_;
    rclcpp::Time score_start_time_;

    double state_machine_dt_;

    //Auto PID control (output fed into manual controller)
    PID xPID_;   //TODO:retune these 0.162 for pixel PID
    PID yPID_;   //TODO:retune these (can also be in pixels depends on which one performs better) 0.0075 for pixel PID
    PID zPID_;   //not used for now due to baro reading malfunction
    PID yawPID_; //can also tune kd with a little overshoot induced
    PID rollPID_;
    PID rollRatePID_;
    PID theta_yPID_;

    ZEstimator z_est_;
    ZEstimator z_est_2;
    EMAFilter z_lowpass_;
    EMAFilter z_lowpass_2;

    Eigen::Matrix3d acc_A_;
    Eigen::Vector3d acc_b_;

    void heartbeat_timer_callback();
    void imu_timer_callback();
    void baro_timer_callback();
    void state_machine_callback();
    void vision_timer_callback();

    void state_machine_manual_callback();
    void state_machine_autonomous_callback();
    void state_machine_searching_callback();
    void state_machine_approach_callback();
    void state_machine_catching_callback();
    void state_machine_caught_callback();
    void state_machine_goalSearch_callback();
    void state_machine_approachGoal_callback();
    void state_machine_scoringStart_callback();
    void state_machine_shooting_callback();
    void state_machine_scored_callback();
    void state_machine_default_callback();

    void calculate_avoidance_from_quadrant(int quadrant);
    std::string auto_state_to_string(autoState state);
    target_type auto_state_to_desired_target_type(autoState state);
    void publish_log(std::string message);
    void auto_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void cal_baro_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void baro_subscription_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void grab_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void kill_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void shoot_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void motor_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void goal_color_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void avoidance_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targets_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void pixels_subscription_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
    float searchDirection();
    bool load_pid_config();
    bool load_acc_calibration();

    TargetData predictTargetPosition(float offset=0.0);
};

#endif
