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
#define L_Pitch                   10                    
#define L_Yaw                     3   // not changed           
#define R_Pitch                   0               
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
    Servo_L.servo_setup(L_Pitch);
    Servo_R.servo_setup(R_Pitch);
    Servo_L.servo_angle(0);
    // Servo_R.servo_angle(180);
    // brushless Brushless_L;
    // brushless Brushless_R;
    // Brushless_L.brushless_setup(5);
    // Brushless_R.brushless_setup(16);
    // Brushless_L.brushless_thrust(1500);
    // Brushless_R.brushless_thrust(1500);
    // int thrust = 1500;
    // int new_thrust = 1500;
    int angle = 0;
    int new_angle = 0;
    // OPI_IMU imu;
    // imu.OPI_IMU_Setup();

    Gimbal leftGimbal;
    Gimbal rightGimbal;

    
    delay(5000);
    while(1){
        cout<<"Servo Angle: "<<endl;
        cin>>new_angle;
        if (new_angle != angle){
            angle = new_angle;
        }
        printf("Angle: %d\n", angle);
        // Brushless_L.brushless_thrust(thrust);
        // Brushless_R.brushless_thrust(thrust);
        Servo_L.servo_angle(angle);
        delay(1000);
    }
    return(1);
}