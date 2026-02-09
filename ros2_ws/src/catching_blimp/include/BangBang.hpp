#ifndef BANG_BANG_HPP
#define BANG_BANG_HPP

// Simple bang-bang (on/off) controller.
// Used to drive a signal toward a setpoint with a deadband instead of
// a smooth PID response (e.g., for coarse goal height positioning).
class BangBang {
    public:
    BangBang(float deadBand, float centeringCom);
    float calculate(float setPoint, float actural);

    private:
    float deadBand;
    float centeringCom;
};

#endif