#include <cmath>
#include "PID.hpp"

using namespace std;

PID::PID():
    _kp(0),
    _ki(0),
    _kd(0),
    _error(0),
    _pre_error(0),
    _integral(0),
    _i_min(-1),
    _i_max(1),
    _d_limit(0),
    _limit_output(false)
{}

/**
 * Implementation
 */
PID::PID(double kp, double ki, double kd) :
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _error(0),
    _pre_error(0),
    _integral(0),
    _i_min(-1),
    _i_max(1),
    _d_limit(0),
    _limit_output(false)
{
}

PID::~PID(){}

void PID::setOutputLimits(double min, double max) {
    _out_min = min;
    _out_max = max;
    _limit_output = true;
}

void PID::setIMin(double iMin) {
    _i_min = iMin;
}

void PID::setIMax(double iMax) {
    _i_max = iMax;
}

void PID::setILimit(double iLimit) {
    _i_min = -1.0*abs(iLimit);
    _i_max = abs(iLimit);
}

void PID::setDLimit(double dLimit) {
    _d_limit = abs(dLimit);
}

void PID::setPGain(double pGain) {
    _kp = pGain;
}

void PID::setIGain(double iGain) {
    _ki = iGain;
}

void PID::setDGain(double dGain) {
    _kd = dGain;
}

double PID::constrain(double x, double a, double b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

double PID::calculate(double setpoint, double pv, double dt) {
    // Calculate error
    _error = setpoint - pv;

    // Proportional term
    double p_out = _kp * _error;

    // Integral term
    if (_ki > 0) {
        _integral += _error * dt;
    }
    double i_out = _ki * _integral;

    //Integral windup limit
    if (abs(i_out) > 0) {
        i_out = constrain(i_out, _i_min, _i_max);
    }

    // Derivative term (zero if dt == 0)
    double d_out = 0;
    if (dt != 0) {
        double derivative = (_error - _pre_error) / dt;
        d_out = _kd * derivative;

        if (_d_limit > 0) {
            d_out = constrain(d_out, -1 * _d_limit, _d_limit);
        }
    }

    // Calculate total output
    double output = p_out + i_out + d_out;

    // Restrict to max/min
    if (_limit_output) {
        output = constrain(output, _out_min, _out_max);
    }

    // Save error to previous error
    _pre_error = _error;

    return output;
}

void PID::reset() {
    _error = 0;
    _pre_error = 0;
    _integral = 0;
}
