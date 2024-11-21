#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include "Servo.hpp"
#include "Brushless.hpp"

class Gimbal {
    public:
        void gimbal_init(int pitchPin, int motorPin, double newDeadband, double newTurnOnCom, 
                            double newMinCom, double newMaxCom, double newThetaOffset, bool thetaSign, bool ccw, double newFilter);
        bool readyGimbal(bool debug, bool motors_off, double roll, double pitch, double up, double forward);
        void updateGimbal(bool ready);
        double getServoAngle();
        double getServoUS();
        double getBrushlessThrust();
        double servoThreshold;

    private:
        double motorCom(double command);

        double deadband_;
        double turnOnCom;
        double minCom;
        double maxCom;
        double thetaOffset;
        double filter;
        double nextMotorCom;

        //attach to pin
        Servo yawServo;
        Servo pitchServo;
        Brushless motor;

        double thetaPos;

        double thetaSign_;
        double thrustSign_;
};

#endif