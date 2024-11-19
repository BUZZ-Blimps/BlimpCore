#include "servo.hpp"
#include <cstdio>


void servo::servo_setup(int PIN){
    this->arr = 1000;
    this->div = 480;

    wiringPiSetup();
    this->pin = PIN;
    pinMode(PIN, PWM_OUTPUT);
    pwmSetRange(PIN, this->arr);
    pwmSetClock(PIN, this->div);
    servo_angle(0);
}

void servo::servo_PIN(int PIN){
    this->pin = PIN;
}

double servo::servo_angle(double angle){
    if (0 <= angle && angle <= 180){
        this->curr_angle = angle;
	    double pwm_val = 50.0/180.0*angle + 50.0;
        pwmWrite(this->pin, pwm_val);
        // printf("Writing %.2f to serv\n", angle);
        return this->curr_angle;
    } else {
        printf("Servo angle out of range!\n");
        return(this->curr_angle);
    }
}

double servo::get_angle() {
    return this->curr_angle;
}

double servo::get_us() {
    //Angle (0-100) to PWM register (50-100) mapping
    double pwm_val = 50.0/180.0*this->curr_angle + 50.0;

    //PWM register to microsecond pulse width
    double us = 20000.0 * pwm_val/this->arr;
    return us;
}