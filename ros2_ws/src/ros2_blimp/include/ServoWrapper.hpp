#ifndef SERVO_WRAPPER_HPP
#define SERVO_WRAPPER_HPP

#include "servo.hpp"
#include "brushless.hpp"

class ServoWrapper{
    public:
    void attach(int servoPin);
    void write(double motorValue);
    double getServo();
    void updateApproximation();
    double motorSpeed;

    private:
    double lastMotorPos;
    double lastValueTime;
    double motorPosition;
    double minMotorPos;
    double maxMotorPos;
    double targetMotorPos;
    double currentMotorPos;

    brushless motor;

};

#endif