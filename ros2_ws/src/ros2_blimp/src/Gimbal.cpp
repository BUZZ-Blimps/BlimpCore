#include <math.h>
#include <cstdio>

#include "Gimbal.hpp"

// Gimbal::Gimbal(double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom, double newPhiOffset, double newFilter){
//   deadband = newDeadband;
//   turnOnCom = newTurnOnCom;
//   minCom = newMinCom;
//   maxCom = newMaxCom;
//   phiOffset = newPhiOffset;
//   filter = newFilter;
//   phiPos1 = phiOffset;
//   thetaPos = 0;
//   servoThreshold = 1000; // (degrees) Defines how close servos must be for brushless motors to activate
// }
void Gimbal::gimbal_init(int yawPin, int pitchPin, int motorPin,double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom, double newPhiOffset, double newFilter){
    deadband = newDeadband;
    turnOnCom = newTurnOnCom;
    minCom = newMinCom;
    maxCom = newMaxCom;
    phiOffset = newPhiOffset;
    filter = newFilter;
    phiPos1 = phiOffset;
    thetaPos = 0;
    servoThreshold = 1000; // (degrees) Defines how close servos must be for brushless motors to activate
    
    // this->yawServo.servo_setup(yawPin);
    // this->yawServo.servo_angle(0);
    this->pitchServo.servo_setup(pitchPin);
    this->pitchServo.servo_angle(phiOffset);

    this->motor.brushless_setup(motorPin);
    this->motor.brushless_thrust(1500);
}

bool Gimbal::readyGimbal(bool debug, bool motors_off, double roll, double pitch, double yaw, double up, double forward) {

  /*
  // Assuming yaw = 0
  yaw = 0;
  double theta = 0;

  if(phiOffset == 45){
    // left gimbal
    forward = -forward;
  }else{
    // right gimbal
  }

  double phi = atan2(up,forward)*180/pi;
  double thrust = sqrt(pow(up,2)+pow(forward,2));

  if (0 <= phi && phi <= 180){
    thrustf = thrust * sqrt(2);
  }else if (-180 <= phi && phi < 0){
    phi = phi + 180;
    thrustf = -thrust * sqrt(2);
  }else{
    // Should not happen
    printf("atan2 error: phi = %.1f deg\n", phi);
  }
  */


  
  // if (debug) Serial.println("Corrected Inputs");
  // if (debug) Serial.print("Forward: ");
  // if (debug) Serial.print(forward);
  // if (debug) Serial.print("\tYaw: ");
  // if (debug) Serial.print(yaw);
  // if (debug) Serial.print("\tUp: ");
  // if (debug) Serial.println(up);
  // self.angle = np.arctan2(np.sin(self.angle), np.cos(self.angle))
  double epsilon = 0.00001;
  double thrust = sqrt(pow(yaw,2)+pow(up,2)+pow(forward,2));
  double theta1 = atan2(yaw,epsilon + forward)*180/pi;
  // theta1 = atan2(sin(theta1), cos(theta1));
  double phi1 = asin(up/thrust)*180/pi;
  double theta2 = atan2(-yaw,-forward)*180/pi;
  // theta2 = atan2(sin(theta2), cos(theta2));
  double phi2 = asin(-up/thrust)*180/pi;
  double theta3 = theta1;
  double phi3 = phi1-180;
  double theta4 = theta2;
  double phi4 = phi2-180;
  // if (debug) Serial.println("Initial Solutions");
  // if (debug) Serial.print(theta1);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(theta2);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(theta3);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.println(theta4);
  // if (debug) Serial.print(phi1);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(phi2);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(phi3);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.println(phi4);
  // if (debug) Serial.println();
  // if (phiOffset == 45){
  //   printf("Left: ");
  // } else {
  //   printf("Right: ");
  // }

  double thetaOffset = 135;
  theta1 += thetaOffset;
  theta2 += thetaOffset;
  theta3 += thetaOffset;
  theta4 += thetaOffset;
  phi1 += phiOffset;
  phi2 += phiOffset;
  phi3 += phiOffset;
  phi4 += phiOffset;

  // printf("yaw: %.1f ", yaw);
  // printf("up: %.1f ", up);
  // printf("forward: %0.1f \n", forward);

  // // printf("Thrust: %.1f ", thrust);
  // printf("Phi1: %.1f ", phi1);
  // printf("Phi2: %.1f ", phi2);
  // printf("Phi3: %.1f ", phi3);
  // printf("Phi4: %0.1f \n", phi4);

  // printf("Theta1: %.1f ", theta1);
  // printf("Theta2: %.1f ", theta2);
  // printf("Theta3: %.1f ", theta3);
  // printf("Theta4: %0.1f \n", theta4);
  // if (debug) Serial.print("Shifted Solutions");
  // if (debug) Serial.print(theta1);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(theta2);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(theta3);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.println(theta4);
  // if (debug) Serial.print(phi1);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(phi2);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(phi3);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.println(phi4);
  // if (debug) Serial.println();
  // double theta = theta1;
  // double phi = phi1;
  double theta;
  double phi;
  double thrustf = thrust*sqrt(2);
  bool sol1 = theta1 > 0 && theta1 < 180 && phi1 > 0 && phi1 < 180;
  bool sol2 = theta2 > 0 && theta2 < 180 && phi2 > 0 && phi2 < 180;
  bool sol3 = theta3 > 0 && theta3 < 180 && phi3 > 0 && phi3 < 180;
  bool sol4 = theta4 > 0 && theta4 < 180 && phi4 > 0 && phi4 < 180;

  // printf("Before Thrustf: %.1f ", thrustf);

  // bool sol1 =  phi1 > 0 && phi1 < 180;
  // bool sol2 =  phi2 > 0 && phi2 < 180;
  // bool sol3 =  phi3 > 0 && phi3 < 180;
  // bool sol4 =  phi4 > 0 && phi4 < 180;
    if (sol1) {
      // printf("First Solution: ");
      theta = theta1;
      phi = phi1;
      thrustf = thrust*sqrt(2);
    } else if (sol2) {
      // printf("Second Solution: ");
      theta = theta2;
      phi = phi2;
      thrustf = -thrust*sqrt(2);
    } else if (sol3) {
      // printf("Third Solution: ");
      theta = theta3;
      phi = phi3;
      thrustf = -thrust*sqrt(2);
    } else if (sol4) {
      // printf("Fourth Solution: ");
      theta = theta4;
      phi = phi4;
      thrustf = thrust*sqrt(2);
    } else {
       // printf("No Solution: \n");
      theta = theta1;
      phi = phi1;
    }

    // printf("After Thrustf: %.1f \n", thrustf);
    
  // if (debug) Serial.print(theta);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.print(phi);
  // if (debug) Serial.print("\t");
  // if (debug) Serial.println(thrustf);


  thetaPos = filter*theta + (1-filter)*thetaPos;
  phiPos1 = filter*phi + (1-filter)*phiPos1;

  if (abs(thrustf) >= deadband/2.0){ // Turn on motors
    // this->yawServo.servo_angle(135);
    // printf("%.1f\n",phi);
    this->pitchServo.servo_angle((phi));
    if (!motors_off) {
      nextMotorCom = motorCom(thrustf); //mator mapping from "-1000 - 1000" to "1000 - 2000"
      //prevent overpowering
      if (nextMotorCom > 2000){
        nextMotorCom = 2000; //max out
      } else if (nextMotorCom < 1000){
        nextMotorCom = 1000; //max out
      }
    }else{
      nextMotorCom=motorCom(0);
      // this->motor.brushless_thrust(motorCom(0)); //write 1500
    }
    //  return (abs(yawServo.get_angle()-thetaPos)<1000) && (abs(pitchServo.get_angle()-phi)<1000);
    return (abs(pitchServo.get_angle()-phi)<1000);
   } else {
      // printf("Idle: %.1f\n",phi);
      nextMotorCom=motorCom(0);
      // this->motor.brushless_thrust(motorCom(0)); //write 1500
      return true; // Anti blocking mechanism
   }
}

void Gimbal::updateGimbal(bool ready){ // Actual turn on command for brushless motors
  if (ready){
    this->motor.brushless_thrust(nextMotorCom);
    // Serial.println(nextMotorCom);
  }else {
    this->motor.brushless_thrust(motorCom(0));
  }
}

double Gimbal::motorCom(double command) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    if (abs(command) <= deadband/2.0) {
        adjustedCom = 1500;
    } else if (command > deadband/2.0) {
        //Serial.println("?");
        double xo1 = deadband/2.0;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(1000-xo1);
        adjustedCom = m1*command-m1*xo1+yo1;
    } else if (command < deadband/2.0) {
        //Serial.println(":");
        double xo2 = -deadband/2.0;
        double yo2 = -turnOnCom+1500;
        double m2 = (yo2-minCom)/(xo2-(-1000));
        adjustedCom = m2*command-m2*xo2+yo2;
    } else {
        //should never happen, but write 1500 anyway for safety
        adjustedCom = 1500;
    }
    // Serial.println(adjustedCom);
    // this->motor.brushless_thrust(adjustedCom);
    return adjustedCom;
}

double Gimbal::getServoAngle() {
  return pitchServo.get_angle();
}

double Gimbal::getServoUS() {
  return pitchServo.get_us();
}