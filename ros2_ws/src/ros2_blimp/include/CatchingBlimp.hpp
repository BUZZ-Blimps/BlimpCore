#ifndef CATCHING_BLIMP_HPP
#define CATCHING_BLIMP_HPP

//C includes
#include <stdio.h>
#include <stdlib.h>

//C++ includes
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <numeric>

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

//Includes for main.cpp
#include "MotorControl.hpp"
#include "OPI_IMU.hpp"
#include "Madgwick_Filter.hpp"
// #include "baro_acc_kf.hpp"
#include "AccelGCorrection.hpp"
#include "PID.hpp"
#include "EMAFilter.hpp"
// #include "Kalman_Filter_Tran_Vel_Est.hpp"
#include "BangBang.hpp"
#include "optical_ekf.hpp"
// #include "gyro_ekf.hpp"
#include "tripleBallGrabber.hpp"
#include "Gimbal.hpp"
#include "ZEstimator.hpp"

#include <wiringPi.h>

//blimp mode
#define BLIMP_COLOR               red      //either red or blue
#define GOAL_COLOR                orange    //either orange or yellow

#define CEIL_HEIGHT_FROM_START    4 //unused 

//debug mode
#define ZERO_MODE                 false
#define GIMBAL_DEBUG              false
#define MOTORS_OFF                true

//optional controllers
#define USE_EST_VELOCITY_IN_MANUAL  false    //use false to turn off the velosity control to see the blimp's behavior 
#define USE_OBJECT_AVOIDENCE      false     //use false to turn off the obstacle avoidance 

//catch search time after one
#define MAX_SEARCH_WAIT_AFTER_ONE     80.0    //max searching 
#define GAME_BALL_WAIT_TIME_PENALTY   0    //should be set to 20, every catch assumed to be 20 seconds long  

//number of catches attempted
#define TOTAL_ATTEMPTS            2    // attempts at catching 
#define MAX_ATTEMPTS              5    //should be set to 5

//flight area parameters
#define CEIL_HEIGHT               12      //m
#define FLOOR_HEIGHT              1.5    //m

#define MAX_HEIGHT                2    //m  (unused)
#define GOAL_HEIGHT               9.5   //m
#define GOAL_HEIGHT_DEADBAND      0.4   //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER    1.4  //m distance for blimp to trigger goal score 	
#define BALL_GATE_OPEN_TRIGGER   2    //m distance for blimp to open the gate 	
#define BALL_CATCH_TRIGGER       1.2  //m distance for blimp to start the open-loop control
#define AVOID_TRIGGER       0.8  //m distance for blimp to start the open-loop control


//object avoidence motor coms
#define FORWARD_AVOID             125  // 25% throttle
#define YAW_AVOID                 10	 // deg/s
#define UP_AVOID                  20   // % throttle 

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -7  // deg/s
#define GAME_BALL_FORWARD_SEARCH  130 // 30% throttle 
#define GAME_BALL_VERTICAL_SEARCH 225  // 45% throttle


#define GAME_BALL_CLOSURE_COM     180  //approaching at 20% throttle cap
#define GAME_BALL_APPROACH_ANGLE  320  //approach magic number (TODO: reset)
#define GAME_BaLL_X_OFFSET        320   //offset magic number (TODO: reset)

#define CATCHING_FORWARD_COM      350  //catching at 50% throttle 
#define CATCHING_UP_COM           50  //damp out pitch

#define CAUGHT_FORWARD_COM        -300  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -40

#define GOAL_YAW_SEARCH           15   
#define GOAL_FORWARD_SEARCH       150  //200 40% throttle
#define GOAL_UP_VELOCITY          450

#define GOAL_CLOSURE_COM          125  //forward command 25% throttle
#define GOAL_X_OFFSET             80  
#define GOAL_APPROACH_ANGLE       70  //height alignment (approach down)

//goal alignment test
#define ALIGNING_YAW_COM           10 //test
#define ALIGNING_FORWARD_COM       100 //test
#define ALIGNING_UP_COM            100 //test
#define ALIGNING_TRANSLATION_COM   300 //test


#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       400 //40% throttle
#define SCORING_UP_COM            80

#define SHOOTING_FORWARD_COM      400  //counter back motion 
#define SHOOTING_UP_COM           100
//counter moment (right now we do want to shoot up because ball sinks)

#define SCORED_FORWARD_COM        -250
#define SCORED_UP_COM             -50

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0
#define BARO_LOOP_FREQ                  50.0
#define STATE_MACHINE_FREQ              30.0
#define OPTICAL_LOOP_FREQ               55.0

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0

#define GYRO_Y_CONSTANT           323.45

//motor timeout before entering lost state
#define TEENSY_WAIT_TIME          2.0

#define BALL_APPROACH_THRESHOLD   2500
#define BALL_CATCH_THRESHOLD      62000

//**************** TEENSY PINOUT ****************//
//old pcb
// #define L_Pitch                   10 // was 2                    
// #define L_Yaw                     3   //not used, not changed           
// #define R_Pitch                   0               
// #define R_Yaw                     9 //not used, was 5                    

// #define L_Pitch_FB                23                    
// #define L_Yaw_FB                  22                  
// #define R_Pitch_FB                21                    
// #define R_Yaw_FB                  20                  

// #define GATE_S                    99  // was 8              

// #define PWM_R                     5              
// #define PWM_G                     98   // was10           
// #define PWM_L                     2      //was 16        


// #define OF_CS                     10    

//new pcb
#define L_Pitch                   5 // was 2                    
#define L_Yaw                     3   //not used, not changed           
#define R_Pitch                   0               
#define R_Yaw                     9 //not used, was 5                    

#define L_Pitch_FB                23                    
#define L_Yaw_FB                  22                  
#define R_Pitch_FB                21                    
#define R_Yaw_FB                  20                  

#define GATE_S                    2  // was 8              

#define PWM_R                     8              
#define PWM_G                     10   // was10           
#define PWM_L                     16     //was 16        


#define OF_CS                     10    
//***********************************************//

//constants
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
    scored
};

enum blimpState {
    manual,
    autonomous,
    lost,
};

// int blimp_state = manual;

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

enum gameballType{
    green,
    pink
};

class CatchingBlimp: public rclcpp::Node {
public:
    CatchingBlimp();
        
private: 
    rclcpp::TimerBase::SharedPtr timer_imu;
    rclcpp::TimerBase::SharedPtr timer_baro;
    rclcpp::TimerBase::SharedPtr timer_state_machine;
    rclcpp::TimerBase::SharedPtr timer_heartbeat;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr state_machine_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baseBarometer_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrateBarometer_subscription;
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

    bool imu_init_, baro_init_;
    double base_baro_, baro_calibration_offset_, cal_baro_;
    double z_hat_;

    //Auto PID control (output fed into manual controller)
    PID xPID_;   //TODO:retune these 0.162 for pixel PID
    PID yPID_;   //TODO:retune these (can also be in pixels depends on which one performs better) 0.0075 for pixel PID
    PID zPID_;   //not used for now due to baro reading malfunction
    PID yawPID_; //can also tune kd with a little overshoot induced

    ZEstimator z_est_;
    EMAFilter z_lowpass_;

    void heartbeat_timer_callback();
    void imu_timer_callback();
    void baro_timer_callback();
    void state_machine_callback();
    void publish_log(std::string message) const;
    void auto_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void calibrateBarometer_subscription_callback(const std_msgs::msg::Bool::SharedPtr msg);
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
};

#endif