#include "brushless.h"
#include <cstdio>


void brushless::brushless_setup(int PIN){
    this->arr = 1000;
    this->div = 480;
    wiringPiSetup();
    this->pin = PIN;
    pinMode(PIN, PWM_OUTPUT);
    pwmSetRange(PIN, this->arr);
    pwmSetClock(PIN, this->div);
    pwmWrite(PIN, 75);
}

void brushless::brushless_PIN(int PIN){
    this->pin = PIN;
}

double brushless::brushless_thrust(double thrust){
    if (1000 <= thrust && thrust <= 2000) {
	this->curr_thrust = thrust;
        double pwm_val = 5.0/100.0*thrust;
        pwmWrite(this->pin, pwm_val);

	printf("Writing %.2f to motor", pwm_val);

        return this->curr_thrust;
    } else {
        printf("Thrust out of range!\n");
        return(this->curr_thrust);
    }
}

double brushless::get_thrust() {
    return this->curr_thrust;
}
