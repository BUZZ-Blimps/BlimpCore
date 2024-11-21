#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "EMAFilter.hpp"

class MotorControl {
    public:
        void update(double forward, double up, double yaw, double roll);
        double upLeft = 0;
        double upRight = 0;
        double forwardLeft = 0;
        double forwardRight = 0;
};

#endif