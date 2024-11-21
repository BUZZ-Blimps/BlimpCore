#include <cstdio>
#include <iostream>
#include <math.h>

#include "Gimbal.hpp"

void Gimbal::gimbal_init(int pitchPin, int motorPin, double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom, 
                          double newThetaOffset, bool thetaSign, bool ccw, double newFilter) {
    deadband_ = newDeadband;
    turnOnCom = newTurnOnCom;
    minCom = newMinCom;
    maxCom = newMaxCom;
    thetaOffset = newThetaOffset;
    filter = newFilter;

    servoThreshold = 1000; // (degrees) Defines how close servos must be for brushless motors to activate
    thetaSign_ = thetaSign ? 1.0 : -1.0;
    thrustSign_ = ccw ? 1.0 : -1.0;
    
    this->pitchServo.setup(pitchPin);
    this->pitchServo.write_angle(thetaOffset);

    this->motor.setup(motorPin);
    this->motor.write_thrust(1500);
}

bool Gimbal::readyGimbal(bool debug, bool motors_off, double roll, double pitch, double up, double forward) {

    double thrust = sqrt(up*up + forward*forward)*sqrt(2);
    double theta = atan2(up, forward)*180.0/M_PI;

    //Apply offset and sign
    theta = thetaOffset + thetaSign_*theta;

    if (theta > 180) {
        theta -= 180;
        thrust = -thrust;
    }

    if (theta < 0) {
        theta += 180;
        thrust = -thrust;
    }

    //Apply CCW/CW thrust mapping
    thrust = thrustSign_*thrust;

    if (abs(thrust) >= deadband_/2.0) { // Turn on motors

      this->pitchServo.write_angle(theta);

      if (!motors_off) {
        nextMotorCom = motorCom(thrust); //mator mapping from "-1000 - 1000" to "1000 - 2000"
        
        //prevent overpowering
        if (nextMotorCom > 2000) {
            nextMotorCom = 2000; //max out
        } else if (nextMotorCom < 1000) {
            nextMotorCom = 1000; //max out
        }
      }else{
        nextMotorCom = motorCom(0);
        // this->motor.brushless_thrust(motorCom(0)); //write 1500
      }
      //  return (abs(yawServo.get_angle()-thetaPos)<1000) && (abs(pitchServo.get_angle()-phi)<1000);
      return (abs(pitchServo.get_angle() - theta) < 1000);
    } else {
        // printf("Idle: %.1f\n",phi);
        nextMotorCom = motorCom(0);
        // this->motor.brushless_thrust(motorCom(0)); //write 1500
        return true; // Anti blocking mechanism
    }
}

// Actual turn on command for brushless motors
void Gimbal::updateGimbal(bool ready) { 
    if (ready) {
        this->motor.write_thrust(nextMotorCom);
    } else {
        this->motor.write_thrust(motorCom(0));
    }
}

double Gimbal::motorCom(double command) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    if (abs(command) <= deadband_/2.0) {
        adjustedCom = 1500;
    } else if (command > deadband_/2.0) {
        double xo1 = deadband_/2.0;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(1000-xo1);
        adjustedCom = m1*command - m1*xo1+yo1;
    } else if (command < deadband_/2.0) {
        double xo2 = -deadband_/2.0;
        double yo2 = -turnOnCom+1500;
        double m2 = (yo2-minCom)/(xo2-(-1000));
        adjustedCom = m2*command - m2*xo2+yo2;
    } else {
        //should never happen, but write 1500 anyway for safety
        adjustedCom = 1500;
    }

    return adjustedCom;
}

double Gimbal::getServoAngle() {
    return pitchServo.get_angle();
}

double Gimbal::getServoUS() {
    return pitchServo.get_us();
}