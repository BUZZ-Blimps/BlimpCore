#include <math.h>

#include "tripleBallGrabber.hpp"

TripleBallGrabber::TripleBallGrabber() {
    // Start closed
    currentAngle = angle_closed;
    targetAngle = currentAngle;
    state_ = state_closed;
    moveRate = moveRate_slow;
    // Start with motor off 
    currentThrust = shooter_off;
    targetThrust = currentThrust;
    
}

void TripleBallGrabber::ballgrabber_init(int servoPin, int motorPin){
    this->servo_.setup(servoPin);
    this->motor_.setup(motorPin);
    this->servo_.write_angle(currentAngle);
    this->motor_.write_thrust(currentThrust);
}

void TripleBallGrabber::openGrabber(int blimp_state) {
    updateMoveRate(blimp_state);

    targetAngle = angle_open;
    currentThrust = shooter_off;
    this->motor_.write_thrust(currentThrust);
    state_ = state_open;
}

bool TripleBallGrabber::is_open() {
    return state_ == state_open && currentAngle == angle_open;
}

void TripleBallGrabber::closeGrabber(int blimp_state) {
    (void) blimp_state; // Gets rid of unused variable warning. Otherwise does nothing. Feel free to delete :)

    //updateMoveRate(blimp_state);
    moveRate = moveRate_fast; // Close fast, regardless of state

    targetAngle = angle_closed;
    targetThrust = shooter_off;
    currentThrust = targetThrust;

    this->motor_.write_thrust(currentThrust); //turn off
    state_ = state_closed;
}

void TripleBallGrabber::update() {
    double currentTime = micros()/1000000.0;
    double elapsedTime = currentTime - lastCommandTime;
    lastCommandTime = currentTime;

    // servo update
    double maxAngleMovement = elapsedTime * moveRate;

    double errorAngle = targetAngle - currentAngle;

    double deltaAngle;

    if (errorAngle > maxAngleMovement) {
        deltaAngle = maxAngleMovement;
    } else if (errorAngle < -maxAngleMovement) {
        deltaAngle = -maxAngleMovement;
    } else {
        // Error is extremely close
        deltaAngle = errorAngle;
    }

    currentAngle += deltaAngle;
    this->servo_.write_angle(round(currentAngle));

    // motor update
    double maxThurstRamp = elapsedTime * 110; //tunable

    double errorThrust = targetThrust - currentThrust;

    double deltaThrust;

    if (errorThrust > maxThurstRamp) {
        deltaThrust = maxThurstRamp;
    } else if (errorThrust < -maxThurstRamp) {
        deltaThrust = -maxThurstRamp;
    } else {
        // Error is extremely close
        deltaThrust = errorThrust;
    }

    currentThrust += deltaThrust;
    this->motor_.write_thrust(round(currentThrust));
}

void TripleBallGrabber::shoot(int blimp_state) {
    updateMoveRate(blimp_state);

    targetAngle = angle_open;
    currentAngle = targetAngle;
    targetThrust = shooter_shooting;
    currentThrust = targetThrust;

    state_ = state_shooting;
    // this->motor_.write_thrust(1800); 
}

//For when you gotta give the ball that GROK GROK
void TripleBallGrabber::suck() {
    currentThrust = shooter_off;
    targetThrust = shooter_sucking;

    state_ = state_sucking;
    // this->motor_.write_thrust(1250);
}

void TripleBallGrabber::updateMoveRate(int blimp_state) {
    if (blimp_state == 0) { // blimpState::manual
        moveRate = moveRate_fast;
    } else {
        moveRate = moveRate_slow;
    }
}