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
#define PWM_G                     16              
#define PWM_L                     2              

#define OF_CS                     10 
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000

int main(){
    servo Servo_L;
    servo Servo_R;
    servo Servo_G;
    Servo_L.servo_setup(L_Pitch);
    Servo_R.servo_setup(R_Pitch);
    Servo_G.servo_setup(PWM_G);
    Servo_L.servo_angle(180);
    Servo_R.servo_angle(0);
    Servo_G.servo_angle(0);
    brushless Brushless_L;
    brushless Brushless_R;
    Brushless_L.brushless_setup(PWM_L);
    Brushless_R.brushless_setup(PWM_R);
    Brushless_L.brushless_thrust(1500);
    Brushless_R.brushless_thrust(1500);

    OPI_IMU imu;
    imu.OPI_IMU_Setup();

    Gimbal leftGimbal;
    Gimbal rightGimbal;

    delay(5000);

    // printf("Initializing Gimbals");
    // leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    // rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

    // delay(5000);

    // printf("Readying Gimbals");
    // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    
    // delay(5000);
    while(1){

        printf("Sweeping gate up in 1 second...\n");
		delay(1000);
		for(float i=0; i<=45; i++){
			float val = i;
			printf("Servo angle: %f\n", val);
            Servo_G.servo_angle(i);
			delay(30);
        }

        printf("Sweeping gate down in 1 second...\n");
		delay(1000);
		for(float i=45; i>=0; i--){
			float val = i;
			printf("Servo angle: %f\n", val);
            Servo_G.servo_angle(i);
			delay(30);
        }

        // printf("Testing Brushless...\n");
		// delay(1000);
		// for(float i=1500; i<=1700; i++){
		// 	float val = i;
		// 	printf("Brushless Thrust: %f\n", val);
        //     Brushless_L.brushless_thrust(i);
		// 	delay(20);
        // }
        // Brushless_L.brushless_thrust(1500);

    //     printf("Sweeping up in 1 second...\n");
	// 	delay(1000);
	// 	for(float i=0; i<=180; i++){
	// 		float val = i;
	// 		printf("Servo angle: %f\n", val);
	// 		Servo_L.servo_angle(180 - i);
    //         Servo_R.servo_angle(i);
	// 		delay(5);
    //     }



    //     printf("Sweeping down in 1 second...\n");
	// 	delay(1000);
	// 	for(float i=0; i<=180; i++){
	// 		float val = i;
	// 		printf("Servo angle: %f\n", 180 - val);
	// 		Servo_L.servo_angle(i);
    //         Servo_R.servo_angle(180 - i);
	// 		delay(5);
    //     }

    //     delay(500);
    //     printf("simming gimbal\n");
    //     delay(1000);
    //     Servo_L.servo_angle(135);
    //     delay(100);
    //     Servo_R.servo_angle(45);
    //     delay(5000);



    //     printf("Going wild in 1 second...\n");
	// 	delay(1000);
	// 	for(float i=0; i<=20; i++){
    //         int lb = 10;
    //         int ub = 170;
    //         int Lval = rand() % (ub - lb + 1) + lb;
    //         int Rval = rand() % (ub - lb + 1) + lb;
	// 		printf("Left Servo angle: %d\n", Lval);
    //         printf("Right Servo angle: %d\n", Rval);
	// 		Servo_L.servo_angle(180 - Lval);
    //         Servo_R.servo_angle(Rval);
    //         int Blb = 1150;
    //         int Bub = 1850;
    //         int BLval = rand() % (Bub - Blb + 1) + Blb;
    //         int BRval = rand() % (Bub - Blb + 1) + Blb;
	// 		printf("Left Brushless thrust: %d\n", BLval);
    //         printf("Right Brushless thrust: %d\n", BRval);
    //         Brushless_L.brushless_thrust(BLval);
    //         Brushless_R.brushless_thrust(BRval);
	// 		delay(1500);
    //     }



    //     printf("Resetting in 3 seconds...\n");
    //     delay(3000);
    //     Servo_L.servo_angle(0);
    //     Servo_R.servo_angle(180);
    //     Brushless_L.brushless_thrust(1500);
    //     Brushless_R.brushless_thrust(1500);
    //     delay(1000);
    }
    return(1);
}