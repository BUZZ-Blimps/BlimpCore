#ifndef TRIPLE_BALL_GRABBER_HPP
#define TRIPLE_BALL_GRABBER_HPP

#include "Servo.hpp"
#include "Brushless.hpp"

class TripleBallGrabber {  
public:
    TripleBallGrabber();
    void ballgrabber_init(int servoPin, int motorPin);
    bool is_open();
    void openGrabber(int blimp_state);      // Opens gate
    void closeGrabber(int blimp_state);     // Closes gate, turns off motor
    void shoot(int blimp_state);            // Opens gate, uses motor to shoot
    void suck();                            // Opens gate, uses motor to suck
    void update();
    void updateMoveRate(int blimp_state);

    double currentAngle = 0; // [deg]
    double targetAngle = 0; // [deg]
    double time_change_angle = 0; // [s]

    double currentThrust = 1500; // write 0
    double targetThrust = 1500; // write 0 (temp)
    double time_change_thrust = 0; // [s]

    enum grabber_state {
        state_closed,
        state_open,
        // state_shooting,
        // state_sucking
    };

    enum shooting_state {
        state_off,
        state_shooting,
        state_sucking
    };

    grabber_state grabber_state_;
    shooting_state shooting_state_;

private:
    Servo servo_;
    Brushless motor_;
    double moveRate;
    double lastCommandTime = 0; // [s]

    const double moveRate_fast = 360.0; // [deg/s]
    const double moveRate_slow = 180.0; // [deg/s]

    const double angle_closed = 0; // [deg]
    const double angle_open = 180; // [deg]

    const double shooter_change_rate_mag_increase = 300; // [#/s]
    const double shooter_change_rate_mag_decrease = 550; // [#/s], we let it change significantly faster when the magnitude is decreasing
    const double shooter_shooting = 1900; // 80% thrust
    const double shooter_sucking = 1100; // 40%  thrust
    const double shooter_off = 1500; // shooter off
};

#endif