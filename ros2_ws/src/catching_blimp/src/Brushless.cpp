#include "Brushless.hpp"
#include <cstdio>
/*
 * KEY VOCAB:
 *
 * PWM (Pulse Width Modulation):
 *   Switching a pin ON/OFF repeatedly to control average power.
 * DUTY (Duty cycle):
 *   Fraction of each period the signal is ON (e.g. % or count).
 * PERIOD:
 *   Length of one full cycle (time or number of ticks).
 * TICKS:
 *   Timer/counter steps used to measure period and duty.
 
 */

void Brushless::setup(int PIN){
    this->arr = 1000; //PWM period in 'Ticks'. The period = 1000 counts
    this->div = 480; //Clock Divisor
    this->pin = PIN; //Stores the GPIO pin used
    //Configures the pin for PWM output (pulse with modulation)
    pinMode(PIN, PWM_OUTPUT); //Configures pin for PWM output
    pwmSetRange(PIN, this->arr); //Applies the period and divisor - generates correct PWM freq. 
    pwmSetClock(PIN, this->div); 
    pwmWrite(PIN, 75); //This sets the initial duty to 75 out of 1000 
}
//Updates which pin is used. 
void Brushless::set_pin(int PIN) {
    this->pin = PIN;
}

double Brushless::write_thrust(double thrust){
    if (thrust >= 1000 && thrust <= 2000) { //range of allowable thrust
	    this->curr_thrust = thrust; 
        double pwm_val = 5.0/100.0*thrust; // scales values
        pwmWrite(this->pin, pwm_val); 
    } else { //if thrust is out of range, returns error and the current thrust. 
        fprintf(stderr, "Thrust out of range!\n");
    }

    return this->curr_thrust;
}
//Returns the last thrust value that was applied in write thrust
double Brushless::get_thrust() {
    return this->curr_thrust;
}
