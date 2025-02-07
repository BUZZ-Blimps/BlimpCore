#ifndef MOTOR_CONTROL_V2_HPP
#define MOTOR_CONTROL_V2_HPP

#include "EMAFilter.hpp"
#include "Brushless.hpp"

class MotorControl_V2 {
    public:
        void motor_init(int motorPinLeftUp, int motorPinLeftForward, int motorPinRightUp, int motorPinRightForward, double newDeadband, double newTurnOnCom, 
                            double newMinCom, double newMaxCom);
        void update(double forward, double up, double yaw, double roll);

    private:
        double motorCom(double command);
        void motorCom(double command, Brushless& motor);
        
        double deadband_;
        double turnOnCom;
        double minCom;
        double maxCom;
        double thetaOffset;
        double filter;
        double nextMotorCom;

        //attach to pin
        Brushless motorLeftUp;
        Brushless motorLeftForward;
        Brushless motorRightUp;
        Brushless motorRightForward;

        double thetaPos;

        double thetaSign_;
        double thrustSign_;
};

#endif