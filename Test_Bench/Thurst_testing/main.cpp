#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include "OPI_IMU.h"
#include "iostream"

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

using namespace std;

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
    int thrust = 1500;
    int new_thrust = 1500;
    // OPI_IMU imu;
    // imu.OPI_IMU_Setup();

    Gimbal leftGimbal;
    Gimbal rightGimbal;

    
    delay(5000);
    while(1){
        cout<<"Thrust value: "<<endl;
        cin>>new_thrust;
        if (new_thrust != thrust){
            thrust = new_thrust;
        }
        printf("thrust: %d\n", thrust);
        Brushless_L.brushless_thrust(thrust);
        Brushless_R.brushless_thrust(thrust);
        delay(1000);
    }
    return(1);
}