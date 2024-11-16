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
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000

#define Test_Left_Servo           1
#define Test_Right_Servo          2
#define Test_Gate_Servo           3
#define Test_Left_Brushless       4
#define Test_Right_Brushless      5
#define Test_Gate_Brushless       6

using namespace std;
int input;
// int new_input = 0;

void Servo(servo& serv) {
    std::cout << "Servo Angle: " <<endl;;
    std::cin >> input;

    serv.servo_angle(input);

}

void Brushless(brushless& brush) {
    std::cout << "Brushless Speed: " <<endl;;
    std::cin >> input;

    brush.brushless_thrust(input);
}


int main(){
    servo Servo_L;
    servo Servo_R;
    servo Servo_G;
    Servo_L.servo_setup(L_Pitch);
    Servo_R.servo_setup(R_Pitch);
    Servo_G.servo_setup(GATE_S);

    Servo_L.servo_angle(0);
    Servo_R.servo_angle(0);
    Servo_G.servo_angle(0);

    brushless Brushless_L;
    brushless Brushless_R;
    brushless Brushless_G;

    Brushless_L.brushless_setup(PWM_L);
    Brushless_R.brushless_setup(PWM_R);
    Brushless_G.brushless_setup(PWM_G);

    Brushless_L.brushless_thrust(1500);
    Brushless_R.brushless_thrust(1500);
    Brushless_G.brushless_thrust(1500);
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
    int motor = 0;
    // OPI_IMU imu;
    // imu.OPI_IMU_Setup();

    Gimbal leftGimbal;
    Gimbal rightGimbal;

    
    delay(5000);
    while(1){
        cout<<"Which Motor? 1 = Servo Left,  2 = Servo Right, 3 = Servo Gate, 4 = Brushless Left, 5 = Brushless Right, 6 = Brushless Gate: "<<endl;
        cin>>motor;
        switch (motor)
        {
            case Test_Left_Servo:
                Servo(Servo_L);
                break;
            case Test_Right_Servo:
                Servo(Servo_R);
                break;
            case Test_Gate_Servo:
                Servo(Servo_G);
                break;
            case Test_Left_Brushless:
                Brushless(Brushless_L);
                break;
            case Test_Right_Brushless:
                Brushless(Brushless_R);
                break;
            case Test_Gate_Brushless:
                Brushless(Brushless_G);
                break;
            default:
                Brushless_L.brushless_thrust(1500);
                Brushless_R.brushless_thrust(1500);
                Brushless_G.brushless_thrust(1500);

                Servo_L.servo_angle(0);
                Servo_R.servo_angle(0);
                Servo_G.servo_angle(0);
                break;
        }

    }
    return(1);
}