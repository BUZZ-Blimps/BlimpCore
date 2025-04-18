#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include "iostream"

#include "Servo.hpp"

#define GIMBAL_DEBUG              false
#define MOTORS_OFF                false
//new pcb
#define L_Pitch                   5 // was 2                    
#define L_Yaw                     3   //not used, not changed           
#define R_Pitch                   0               
#define R_Yaw                     9 //not used, was 5                    

#define L_Pitch_FB                23                    
#define L_Yaw_FB                  22                  
#define R_Pitch_FB                21                    
#define R_Yaw_FB                  20                  

#define GATE_S                    0  // was 8              

#define PWM_R                     8              
#define PWM_G                     10   // was10           
#define PWM_L                     16     //was 16        


#define OF_CS                     10    
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000

using namespace std;

int main(){
    // servo Servo_L;
    // servo Servo_R;
    Servo Servo_G;
    // Servo_L.servo_setup(L_Pitch);
    // Servo_R.servo_setup(R_Pitch);
    Servo_G.setup(GATE_S);
    // Servo_G.set_pin(GATE_S);

    // Servo_L.servo_angle(0);
    // Servo_R.servo_angle(0);
    Servo_G.write_angle(0);
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

    
    delay(1000);
    while(1){
        cout<<"Servo Angle: "<<endl;
        cin>>new_angle;
        if (new_angle != angle){
            angle = new_angle;
        }
        printf("Angle: %d\n", angle);
        // Brushless_L.brushless_thrust(thrust);
        // Brushless_R.brushless_thrust(thrust);
        Servo_G.write_angle(angle);
        delay(1000);
    }
    return(1);
}