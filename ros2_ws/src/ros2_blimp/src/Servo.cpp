#include <cstdio>

#include "Servo.hpp"

void Servo::setup(int PIN){
    this->arr = 1000;
    this->div = 480;
    this->pin = PIN;

    pinMode(PIN, PWM_OUTPUT);
    pwmSetRange(PIN, this->arr);
    pwmSetClock(PIN, this->div);

    write_angle(0);
}

void Servo::set_pin(int PIN) {
    this->pin = PIN;
}

void Servo::write_microseconds(double us) {
    double pwm_val = 5.0/100.0*us;
    pwmWrite(this->pin, pwm_val);
}

double Servo::write_angle(double angle) {
    //Compute constrained angle and map it to microseconds
    double con_angle = math_helpers::constrain(angle, 0, 180);
    double us = math_helpers::map(con_angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

    //Write microsecond pulse width value to servo
    write_microseconds(us);

    this->curr_angle = con_angle;
    return this->curr_angle;
}

double Servo::get_angle() {
    return this->curr_angle;
}

double Servo::get_us() {
    //Angle (0-100) to PWM register (50-100) mapping
    // double pwm_val = 50.0/180.0*this->curr_angle + 50.0;

    // //PWM register to microsecond pulse width
    // double us = 20000.0 * pwm_val/this->arr;
    
    return math_helpers::map(this->curr_angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);;
}
