/**
 * @file EMAFilter.cpp
 * @brief Exponential moving average (EMA) filter: y_new = alpha * current + (1 - alpha) * y_old.
 *        alpha in (0, 1]; larger alpha = more weight on new sample (less smoothing).
 */

#include "EMAFilter.hpp"
#include <math.h>

// Alpha and last output; first filter() call seeds with current input
EMAFilter::EMAFilter(double newAlpha) {
    this->alpha = newAlpha;
    this->last = 0;
}

// Default: alpha=1 (passthrough), not initialized so first filter() seeds from input
EMAFilter::EMAFilter() {
    this->alpha = 1;
    this->last = 0;
    this->initialized = false;
}

// Next filter() will re-seed with the next input (like first run)
void EMAFilter::reset() {
    this->initialized = false;
}

void EMAFilter::setAlpha(double newAlpha) {
    this->alpha = newAlpha;
}

// Set internal state so next output blends from this value (optional warm start)
void EMAFilter::setInitial(double initial) {
    this->last = initial;
}

// One step: out = alpha*current + (1-alpha)*last. NaN input is returned without updating state.
double EMAFilter::filter(double current) {
    if (std::isnan(current)) {
        return current;
    }

    double next;
    if (!this->initialized) {
        next = current;
        this->initialized = true;
    } else {
        next = (this->alpha) * current + ((double)1.0 - (this->alpha)) * (this->last);
    }

    this->last = next;
    return next;
}
