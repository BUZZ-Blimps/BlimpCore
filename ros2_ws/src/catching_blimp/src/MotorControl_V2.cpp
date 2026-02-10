/**
 * @file MotorControl_V2.cpp
 * @brief Four-motor mixer: forward/up/yaw/roll -> per-motor PWM with deadband and scaling.
 */

#include <cstdio>
#include <iostream>
#include <math.h>

#include "MotorControl_V2.hpp"
//Sets up the four motors and stores the turning parameters
void MotorControl_V2::motor_init(int motorPinLeftUp, int motorPinLeftForward, int motorPinRightUp, int motorPinRightForward, double newDeadband, double newTurnOnCom, 
                            double newMinCom, double newMaxCom) {
    deadband_ = newDeadband; // no thrust change
    turnOnCom = newTurnOnCom;
    minCom = newMinCom;
    maxCom = newMaxCom; // min and max thrust
    //one pin per motor
    this->motorLeftUp.setup(motorPinLeftUp);
    this->motorLeftUp.write_thrust(1500);

    this->motorLeftForward.setup(motorPinLeftForward);
    this->motorLeftForward.write_thrust(1500);

    this->motorRightUp.setup(motorPinRightUp);
    this->motorRightUp.write_thrust(1500);

    this->motorRightForward.setup(motorPinRightForward);
    this->motorRightForward.write_thrust(1500);
}

//converts the body commands into four motor commands and send them 
void MotorControl_V2::update(double forward, double up, double yaw, double roll) {
    double leftForward = forward - yaw; //negative yaw -> turn left forward, less right forward 
    double rightForward = forward + yaw; //positive yaw -> turn right forward, less left forward
    double leftUp = up + roll;
    double rightUp = up - roll;

    double leftFwdCom = motorCom(leftForward, motorLeftForward);
    double rightFwdCom = motorCom(rightForward, motorRightForward);
    double leftUpCom = motorCom(leftUp, motorLeftUp);
    double rightUpCom = motorCom(rightUp, motorRightUp);

    // fprintf(stdout, "Left Fwd: %.2f, Right Fwd: %.2f, Left Up: %.2f, Right Up: %.2f\n", leftFwdCom, rightFwdCom, leftUpCom, rightUpCom);
}
//Turn a signed controller command (-1000 to +1000) into ESC thrust with deadband and min/max -> sent to motor

/*
MotorControl takes commands such as forward, up, yaw roll and 
mixes them into four per-motor commands (left/right, forward/up)
it converts each command to ESC thrust with deadband min/max and 
sends those thrusts to four brushless ESC objects. 
*/
double MotorControl_V2::motorCom(double command, Brushless& motor) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    if (abs(command) <= deadband_/2.0) { //small stick movements dont move motors
        adjustedCom = 1500;
    } else if (command > deadband_/2.0) { //Ramps the motor to max linearly
        // command is positive and outside of deadband
        double xo1 = deadband_/2.0; 
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(500-xo1);

        adjustedCom = m1*command - m1*xo1 + yo1;
    } else if (command < deadband_/2.0) { //decreasing the motor to min linearly
        // command is negative and outside of deadband
        double xo2 = -deadband_/2.0;
        double yo2 = -turnOnCom+1500; 
        double m2 = (yo2-minCom)/(xo2+500);

        adjustedCom = m2*command - m2*xo2 + yo2;
    } else {
        //should never happen, but write 1500 anyway for safety
        adjustedCom = 1500;
    }

    // project back to the admissible range if the input is out of range
    if (adjustedCom >= 2000){
        adjustedCom = 2000;
    } else if (adjustedCom <= 1000){
        adjustedCom = 1000;
    }

    motor.write_thrust(adjustedCom);

    return adjustedCom;
}