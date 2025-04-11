#ifndef SERVO_HPP
#define SERVO_HPP

#include "wiringPi.h"

#include "math_helpers.hpp"

#define MIN_PULSE_WIDTH      1000     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo

class Servo {
public:
    void setup(int PIN);
    void write_microseconds(double us);
    double write_angle(double angle);
    void set_pin(int PIN);
    double get_angle();
    double get_us();

private:
    double curr_angle;
    int pin;
    unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
	unsigned int arr;		// Auto-Reload Register (Period)
	unsigned int div;
	unsigned int div_stepping;
};

#endif
