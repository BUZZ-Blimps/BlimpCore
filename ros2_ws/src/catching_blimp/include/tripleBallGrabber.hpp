#ifndef TRIPLE_BALL_GRABBER_HPP
#define TRIPLE_BALL_GRABBER_HPP

#include "Servo.hpp"
#include "Brushless.hpp"

class TripleBallGrabber {  
public:
    TripleBallGrabber();
    void ballgrabber_init(int servoPin, int motorPin);
    bool is_open();
    void openGrabber(int blimp_state);
    void closeGrabber(int blimp_state);
    void update();
    void shoot(int blimp_state);
    void updateMoveRate(int blimp_state);
    void suck();

    double currentAngle = 0; // [deg]
    double targetAngle = 0; // [deg]

    enum grabber_state {
        state_closed,
        state_open,
        state_shooting,
        state_sucking
    };

    grabber_state state_;

private:
    Servo servo_;
    Brushless motor_;
    double moveRate;
    double lastCommandTime = 0; // [s]

    const double moveRate_fast = 180.0; // [deg/s]
    const double moveRate_slow = 45.0; // [deg/s]

    const double angle_closed = 0; // [deg]
    const double angle_open = 100; // [deg]
};

#endif