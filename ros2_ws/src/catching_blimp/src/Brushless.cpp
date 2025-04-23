#include "Brushless.hpp"
#include <cstdio>

void Brushless::setup(int PIN){
    this->arr = 1000;
    this->div = 480;
    this->pin = PIN;

    pinMode(PIN, PWM_OUTPUT);
    pwmSetRange(PIN, this->arr);
    pwmSetClock(PIN, this->div);
    pwmWrite(PIN, 75);
}

void Brushless::set_pin(int PIN) {
    this->pin = PIN;
}

double Brushless::write_thrust(double thrust){
    if (thrust >= 1000 && thrust <= 2000) {
	    this->curr_thrust = thrust;
        double pwm_val = 5.0/100.0*thrust;
        pwmWrite(this->pin, pwm_val);
    } else {
        fprintf(stderr, "Thrust out of range!\n");
    }

    return this->curr_thrust;
}

double Brushless::get_thrust() {
    return this->curr_thrust;
}
