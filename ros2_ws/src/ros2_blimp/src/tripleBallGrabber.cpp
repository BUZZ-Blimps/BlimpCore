#include <math.h>

#include "tripleBallGrabber.hpp"

TripleBallGrabber::TripleBallGrabber() {
    // Start closed
    currentAngle = angle_closed;
    targetAngle = currentAngle;
    state_ = state_closed;
    moveRate = moveRate_slow;
}

void TripleBallGrabber::ballgrabber_init(int servoPin, int motorPin){
    this->servo_.setup(servoPin);
    this->motor_.setup(motorPin);
    this->servo_.write_angle(currentAngle);
    this->motor_.write_thrust(1500);
}

void TripleBallGrabber::openGrabber(int blimp_state) {
    updateMoveRate(blimp_state);

    targetAngle = angle_open;
    this->motor_.write_thrust(1500);
    state_ = state_open;
}

bool TripleBallGrabber::is_open() {
    return state_ == state_open && currentAngle == angle_open;
}

void TripleBallGrabber::closeGrabber(int blimp_state) {
    //updateMoveRate(blimp_state);
    moveRate = moveRate_fast; // Close fast, regardless of state

    targetAngle = angle_closed;
    this->motor_.write_thrust(1500);
    state_ = state_closed;
}

void TripleBallGrabber::update() {
    double currentTime = micros()/1000000.0;
    double elapsedTime = currentTime - lastCommandTime;
    lastCommandTime = currentTime;

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
}

void TripleBallGrabber::shoot(int blimp_state) {
    updateMoveRate(blimp_state);

    targetAngle = angle_open;
    currentAngle = targetAngle;
    state_ = state_shooting;
    this->motor_.write_thrust(2000);
}

//For when you gotta give the ball that GROK GROK with your low-hanging sack
void TripleBallGrabber::suck() {
    state_ = state_sucking;
    this->motor_.write_thrust(1000);
}

void TripleBallGrabber::updateMoveRate(int blimp_state) {
    if (blimp_state == 0) { // blimpState::manual
        moveRate = moveRate_fast;
    } else {
        moveRate = moveRate_slow;
    }
}