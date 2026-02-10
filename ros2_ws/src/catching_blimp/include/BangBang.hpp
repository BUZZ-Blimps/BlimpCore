/**
 * @file BangBang.hpp
 * @brief Bang-bang controller with deadband; used for goal height positioning.
 */

#ifndef BANG_BANG_HPP
#define BANG_BANG_HPP

/*
The BangBang Controller decides when the controller goes
full one direction, the other direction, or off. 
*/

// Set Point -
// Actual - Measured value from sensor 
// DeadBand -
// CenteringCom - Strength of corrective command (e.g thruster)


class BangBang {
    public:
    BangBang(float deadBand, float centeringCom);

    
    float calculate(float setPoint, float actual);

    private:
    float deadBand;
    float centeringCom;
};

#endif