#ifndef BRUSHLESS_HPP
#define BRUSHLESS_HPP

#include "wiringPi.h"

class Brushless{
    public:
    void setup(int PIN);
    double write_thrust(double thrust);
    void set_pin(int PIN);
    double get_thrust();

    private:
    double curr_thrust;
    int pin;
    unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
	unsigned int arr;		// Auto-Reload Register (Period)
	unsigned int div;
	unsigned int div_stepping;
};

#endif