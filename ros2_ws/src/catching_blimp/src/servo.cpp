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
    pwmWrite(PIN, 75);
}

void servo::servo_PIN(int PIN){
    this->pin = PIN;
}

double servo::servo_angle(double angle){
    if (0 <= angle && angle <= 180){
        this->curr_angle = angle;
	double pwm_val = 50.0/180.0*angle + 50.0;
        pwmWrite(this->pin, pwm_val);
        return this->curr_angle;
    } else {
        printf("Servo angle out of range!\n");
        return(this->curr_angle);
    }
}

double servo::get_angle(){
    return this->curr_angle;
}
