#include <math.h>

#include "tripleBallGrabber.hpp"

TripleBallGrabber::TripleBallGrabber() {
    // Start at slow speed
    moveRate = moveRate_slow;

    // Get current time
    double current_time = micros()/1000000.0;

    // Start closed
    currentAngle = angle_closed;
    targetAngle = currentAngle;
    grabber_state_ = state_closed;
    time_change_angle = current_time;

    // Start with motor off 
    currentThrust = shooter_off;
    targetThrust = currentThrust;
    shooting_state_ = state_off;
    time_change_thrust = current_time;
    
}

void TripleBallGrabber::ballgrabber_init(int servoPin, int motorPin){
    this->servo_.setup(servoPin);
    this->motor_.setup(motorPin);
    this->servo_.write_angle(currentAngle);
    this->motor_.write_thrust(currentThrust);
}

bool TripleBallGrabber::is_open() {
    return grabber_state_ == state_open && currentAngle == angle_open;
}

// Opens gate
void TripleBallGrabber::openGrabber(int blimp_state) {
    double current_time = micros()/1000000.0;
    
    updateMoveRate(blimp_state);

    if(grabber_state_ != state_open) time_change_angle = current_time;
    grabber_state_ = state_open;
    targetAngle = angle_open;
}

// Closes gate, turns off motor
void TripleBallGrabber::closeGrabber(int blimp_state) {
    (void) blimp_state; // Gets rid of unused variable warning. Otherwise does nothing. Feel free to delete :)
    double current_time = micros()/1000000.0;

    //updateMoveRate(blimp_state);
    moveRate = moveRate_fast; // Close fast, regardless of state

    if(grabber_state_ != state_closed) time_change_angle = current_time;
    grabber_state_ = state_closed;    
    targetAngle = angle_closed;

    if(shooting_state_ != state_off) time_change_thrust = current_time;
    shooting_state_ = state_off;
    targetThrust = shooter_off;
}

// Opens gate, uses motor to shoot
void TripleBallGrabber::shoot(int blimp_state) {
    updateMoveRate(blimp_state);
    double current_time = micros()/1000000.0;

    if(grabber_state_ != state_open) time_change_angle = current_time;
    grabber_state_ = state_open;
    targetAngle = angle_open;

    if(shooting_state_ != state_shooting) time_change_thrust = current_time;
    shooting_state_ = state_shooting;
    targetThrust = shooter_shooting;
}

//For when you gotta give the ball that GROK GROK
// Opens gate, uses motor to suck
void TripleBallGrabber::suck() {
    double current_time = micros()/1000000.0;

    if(grabber_state_ != state_open) time_change_angle = current_time;
    grabber_state_ = state_open;
    targetAngle = angle_open;

    if(shooting_state_ != state_sucking) time_change_thrust = current_time;
    shooting_state_ = state_sucking;
    targetThrust = shooter_sucking;
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
    double shooter_change_rate;
    if(fabs(targetThrust-shooter_off) > fabs(currentThrust-shooter_off)){
        // Magnitude is increasing
        shooter_change_rate = shooter_change_rate_mag_increase;
    }else{
        shooter_change_rate = shooter_change_rate_mag_decrease;
    }

    double maxThurstRamp = elapsedTime * shooter_change_rate; //tunable

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


void TripleBallGrabber::updateMoveRate(int blimp_state) {
    if (blimp_state == 0) { // blimpState::manual
        moveRate = moveRate_fast;
    } else {
        moveRate = moveRate_slow;
    }
}