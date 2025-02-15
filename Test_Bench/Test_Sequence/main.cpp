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
#define L_Pitch                   5                        
#define R_Pitch                   0




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


int main() {
    servo Servo_L;
    servo Servo_R;
    // servo Servo_G;
    Servo_L.servo_setup(L_Pitch);
    Servo_R.servo_setup(R_Pitch);
    // Servo_G.servo_setup(GATE_S);
    Servo_L.servo_angle(180);
    Servo_R.servo_angle(0);
    // Servo_G.servo_angle(0);

    brushless Brushless_L1;
    brushless Brushless_R1;
    brushless Brushless_L2;
    brushless Brushless_R2;

    Brushless_L1.brushless_setup(PWM_L);
    Brushless_R1.brushless_setup(PWM_R);
    Brushless_L2.brushless_setup(GATE_S);
    Brushless_R2.brushless_setup(PWM_G);

    Brushless_L1.brushless_thrust(1500);
    Brushless_R1.brushless_thrust(1500);
    Brushless_L2.brushless_thrust(1500);
    Brushless_R2.brushless_thrust(1500);
    // Gimbal leftGimbal;
    // Gimbal rightGimbal; 

    // leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    // rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

    OPI_IMU imu;
    imu.OPI_IMU_Setup();

    printf("Checking IMU...\n");

    // //Read 100 IMU samples
    // for (int i = 0; i < 100; i++) {
    //     imu.IMU_read();

    //     // fprintf(stdout, "Gyro: %.2f, %.2f, %.2f\n", imu.gyr_rateXraw, imu.gyr_rateYraw, imu.gyr_rateZraw);
    //     // fprintf(stdout, "Acc: %.2f, %.2f, %.2f\n", imu.AccXraw, imu.AccYraw, imu.AccZraw);

    //     delay(10);
    // }


    printf("Done!\n");

    // Gimbal leftGimbal;
    // Gimbal rightGimbal;

     delay(1000);

    // delay(1000);

    // printf("Initializing Gimbals");
    // leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    // rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

    // delay(5000);

    // printf("Readying Gimbals");
    // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
    
    // delay(5000);

    //Quick stress test
    // Brushless_L.brushless_thrust(1850);
    // Brushless_R.brushless_thrust(1850);
    // Brushless_G.brushless_thrust(1850);
    // delay(1000);
    // Brushless_L.brushless_thrust(1150);
    // Brushless_R.brushless_thrust(1150);
    // Brushless_G.brushless_thrust(1150);
    // delay(1000);
    // Brushless_L.brushless_thrust(1850);
    // Brushless_R.brushless_thrust(1850);
    // Brushless_G.brushless_thrust(1850);
    // delay(1000);
    // Brushless_L.brushless_thrust(1150);
    // Brushless_R.brushless_thrust(1150);
    // Brushless_G.brushless_thrust(1150);
    // delay(1000);
    // Brushless_L.brushless_thrust(1500);
    // Brushless_R.brushless_thrust(1500);
    // Brushless_G.brushless_thrust(1500);
    // delay(1000);



    while(1) {


        // printf("Sweeping gate up in 1 second...\n");
		// delay(1000);
		// for(float i=0; i<=180; i++){
		// 	float val = i;
		// 	printf("Servo angle: %f\n", val);
        //   Servo_G.servo_angle(i);
		// 	delay(30);
        // }

        // printf("Sweeping gate down in 1 second...\n");
		// delay(1000);
		// for(float i=180; i>=0; i--){
		// 	float val = i;
		// printf("Servo angle: %f\n", val);
        //     Servo_G.servo_angle(i);
		// 	delay(30);
        // }

        Brushless_L1.brushless_thrust(1500);
        Brushless_R1.brushless_thrust(1500);
        Brushless_L2.brushless_thrust(1500);
        Brushless_R2.brushless_thrust(1500);
        delay(3000);

        //Back Thrust
        // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -320.0, -0.0);
        // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 320.0, -0.0);
        // leftGimbal.updateGimbal(leftReady && rightReady);
        // rightGimbal.updateGimbal(leftReady && rightReady);

        //Down Thrust
        // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -0.0, -320.0);
        // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -0.0, -320.0);
        // leftGimbal.updateGimbal(leftReady && rightReady);
        // rightGimbal.updateGimbal(leftReady && rightReady);

        
        //Up Thrust 
        // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -0.0, 320.0);
        // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -0.0, 320.0);
        // leftGimbal.updateGimbal(leftReady && rightReady);
        // rightGimbal.updateGimbal(leftReady && rightReady);

        // Forward Thrust
        // bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 500.0, -0.0);
        // bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, -500.0, -0.0);
        // leftGimbal.updateGimbal(leftReady && rightReady);
        // rightGimbal.updateGimbal(leftReady && rightReady);


        // delay(33);

        printf("Testing R/L Brushless...\n");
        delay(2000);
        for(int i=1500; i<=1650; i++){
            printf("Brushless Thrust: %d\n", i);
            Brushless_L1.brushless_thrust(i);
            Brushless_R1.brushless_thrust(i);
            Brushless_L2.brushless_thrust(i);
            Brushless_R2.brushless_thrust(i);
            
            delay(50);
        }
        Brushless_L1.brushless_thrust(1500);
        Brushless_R1.brushless_thrust(1500);
        Brushless_L2.brushless_thrust(1500);
        Brushless_R2.brushless_thrust(1500);
        delay(5000);


        printf("Testing R/L Brushless backwards...\n");
        delay(2000);
        for(int i=1500; i>=1350; i--) {
            printf("Brushless Thrust: %d\n", i);
            Brushless_L1.brushless_thrust(i);
            Brushless_R1.brushless_thrust(i);
            Brushless_L2.brushless_thrust(i);
            Brushless_R2.brushless_thrust(i);

            delay(50);
        }  
        Brushless_L1.brushless_thrust(1500);
        Brushless_R1.brushless_thrust(1500);
        Brushless_L2.brushless_thrust(1500);
        Brushless_R2.brushless_thrust(1500);
        delay(5000);


        // printf("Testing Gate Brushless...\n");
        // for(int i=1500; i<=1750; i++){
        //     printf("Brushless Thrust: %d\n", i);
        //     Brushless_G.brushless_thrust(i);
        //     delay(20);
        // }
        // Brushless_G.brushless_thrust(1500);

        // printf("Sweeping RLG servo up in 1 second...\n");
	 	// delay(1000);
	 	// for (int i=0; i<=180; i++){
        //     printf("Servo angle: %d\n", i);
        //     Servo_L.servo_angle(180 - i);
        //     Servo_R.servo_angle(i);
        //     Servo_G.servo_angle(i);
        //     delay(5);
        // }

        // printf("Sweeping RLG servo down in 1 second...\n");
	 	// delay(1000);
	 	// for (int i=0; i<=180; i++){
	 	// 	printf("Servo angle: %d\n", 180 - i);
	 	// 	Servo_L.servo_angle(i);
        //     Servo_R.servo_angle(180 - i);
        //     Servo_G.servo_angle(180 - i);
	 	// 	delay(5);
        //  }

    //     delay(500);
    //     printf("simming gimbal\n");
    //     delay(1000);
    //     Servo_L.servo_angle(135);
    //     delay(100);
    //     Servo_R.servo_angle(45);
    //     delay(5000);



    // //     printf("Going wild in 1 second...\n");
	// // 	delay(1000);
	// // 	for(float i=0; i<=20; i++){
    // //         int lb = 10;
    // //         int ub = 170;
    // //         int Lval = rand() % (ub - lb + 1) + lb;
    // //         int Rval = rand() % (ub - lb + 1) + lb;
	// // 		printf("Left Servo angle: %d\n", Lval);
    // //         printf("Right Servo angle: %d\n", Rval);
	// // 		Servo_L.servo_angle(180 - Lval);
    // //         Servo_R.servo_angle(Rval);
    // //         int Blb = 1150;
    // //         int Bub = 1850;
    // //         int BLval = rand() % (Bub - Blb + 1) + Blb;
    // //         int BRval = rand() % (Bub - Blb + 1) + Blb;
	// // 		printf("Left Brushless thrust: %d\n", BLval);
    // //         printf("Right Brushless thrust: %d\n", BRval);
    // //         Brushless_L.brushless_thrust(BLval);
    // //         Brushless_R.brushless_thrust(BRval);
	// // 		delay(1500);
    // //     }



        printf("Resetting in 3 seconds...\n");
        // Servo_L.servo_angle(0);
        // Servo_R.servo_angle(180);
        // Servo_G.servo_angle(180);
        Brushless_L1.brushless_thrust(1500);
        Brushless_R1.brushless_thrust(1500);
        Brushless_L2.brushless_thrust(1500);
        Brushless_R2.brushless_thrust(1500);
        delay(3000);
    }
    return(1);
}
