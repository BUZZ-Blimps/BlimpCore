#include <cstdio>
#include <iostream>
#include <math.h>

#include "MotorControl_V2.hpp"

void MotorControl_V2::motor_init(int motorPinLeftUp, int motorPinLeftForward, int motorPinRightUp, int motorPinRightForward, double newDeadband, double newTurnOnCom, 
                            double newMinCom, double newMaxCom) {
    deadband_ = newDeadband;
    turnOnCom = newTurnOnCom;
    minCom = newMinCom;
    maxCom = newMaxCom;
    
    this->motorLeftUp.setup(motorPinLeftUp);
    this->motorLeftUp.write_thrust(1500);

    this->motorLeftForward.setup(motorPinLeftForward);
    this->motorLeftForward.write_thrust(1500);

    this->motorRightUp.setup(motorPinRightUp);
    this->motorRightUp.write_thrust(1500);

    this->motorRightForward.setup(motorPinRightForward);
    this->motorRightForward.write_thrust(1500);
}

void MotorControl_V2::update(double forward, double up, double yaw, double roll) {
    double leftForward = forward - yaw;
    double rightForward = forward + yaw;
    double leftUp = up + roll;
    double rightUp = up - roll;

    double leftFwdCom = motorCom(leftForward, motorLeftForward);
    double rightFwdCom = motorCom(rightForward, motorRightForward);
    double leftUpCom = motorCom(leftUp, motorLeftUp);
    double rightUpCom = motorCom(rightUp, motorRightUp);

    // fprintf(stdout, "Left Fwd: %.2f, Right Fwd: %.2f, Left Up: %.2f, Right Up: %.2f\n", leftFwdCom, rightFwdCom, leftUpCom, rightUpCom);
}

double MotorControl_V2::motorCom(double command, Brushless& motor) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    if (abs(command) <= deadband_/2.0) {
        adjustedCom = 1500;
    } else if (command > deadband_/2.0) {
        // command is positive and outside of deadband
        double xo1 = deadband_/2.0;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(500-xo1);

        adjustedCom = m1*command - m1*xo1 + yo1;
    } else if (command < deadband_/2.0) {
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