#include "MotorControl.hpp"

void MotorControl::update(double forward, double up, double yaw, double roll) {
    this->forwardLeft = forward - yaw;
    this->forwardRight = forward + yaw;
    this->upLeft = up + roll;
    this->upRight = up - roll;
}
