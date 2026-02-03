/**
 * @file math_helpers.hpp
 * @brief Utility: constrain(value, min, max) and map(x, in_min, in_max, out_min, out_max).
 */

#ifndef MATH_HELPERS_HPP
#define MATH_HELPERS_HPP

namespace math_helpers {
    double constrain(double input, double out_min, double out_max);
    double map(double x, double in_min, double in_max, double out_min, double out_max);
}

#endif