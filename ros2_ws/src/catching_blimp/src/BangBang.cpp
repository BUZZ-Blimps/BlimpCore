#include "BangBang.hpp"

BangBang::BangBang(float newDeadBand, float newCenteringCom) {
    this->deadBand = newDeadBand;
    this->centeringCom = newCenteringCom;
}
//Let error = acutal - setPoint 

/*
if the error > deadBand, then the actual is too high above the setpoint
    -> so we return -centeringCom to push it down
    
if error < -deadBand, then the actual is too low below the setpoint
    -> so we return +centeringCom to push it up 

The output is one of these three values: 
    - +centeringCom (correct up)
    - -centeringCom (correct down)
    - or 0 (acceptable value)
*/

float BangBang::calculate(float setPoint, float actual) {
    
    if (actual - setPoint > this->deadBand) {
        return -this->centeringCom;
    } else if (actual - setPoint < -this->deadBand) {
        return this->centeringCom;
    } else {
        return 0;
    }
}