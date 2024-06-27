#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include "OPI_IMU.h"

#include "servo.h"
#include "brushless.h"
#include "OPI_IMU.h"
#include "Gimbal.h"

#define GIMBAL_DEBUG              false
#define MOTORS_OFF                false
#define L_Pitch                   0                    
#define L_Yaw                     3   // not changed           
#define R_Pitch                   2               
#define R_Yaw                     9 //was 5              

#define L_Pitch_FB                23                    
#define L_Yaw_FB                  22                  
#define R_Pitch_FB                21                    
#define R_Yaw_FB                  20                  

#define GATE_S                    8                

#define PWM_R                     5              
#define PWM_G                     10              
#define PWM_L                     16              

#define OF_CS                     10 
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000

int main(){
    servo Servo_L;
    servo Servo_R;
    Servo_L.servo_setup(0);
    Servo_R.servo_setup(2);
    Servo_L.servo_angle(0);
    Servo_R.servo_angle(180);
    brushless Brushless_L;
    brushless Brushless_R;
    Brushless_L.brushless_setup(5);
    Brushless_R.brushless_setup(16);
    Brushless_L.brushless_thrust(1500);
    Brushless_R.brushless_thrust(1500);

    OPI_IMU imu;
    imu.OPI_IMU_Setup();

    Gimbal leftGimbal;
    Gimbal rightGimbal;

    delay(5000);

    printf("Initializing Gimbals");
    leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

    delay(5000);

    printf("Readying Gimbals");
    bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    
    delay(5000);
    while(1){

        	Brushless_L.brushless_thrust(1700);
            Brushless_R.brushless_thrust(1700);

        delay(3000);
    }
    return(1);
}