#include "math_helpers.hpp"

#include <iostream>

namespace math_helpers {
    double constrain(double input, double out_min, double out_max) {
        double ret = input;

        if (ret < out_min) {
            ret = out_min;
        } else if (ret > out_max) {
            ret = out_max;
        }

        return ret;
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
