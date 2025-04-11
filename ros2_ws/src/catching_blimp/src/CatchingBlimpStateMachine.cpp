#include "CatchingBlimp.hpp"

void CatchingBlimp::state_machine_callback() {
    vision_timer_callback();
    
    rclcpp::Time now = this->get_clock()->now();
    state_machine_dt_ = (now - state_machine_time_).seconds();
    state_machine_time_ = now;

    auto debug_msg = std_msgs::msg::Float64MultiArray();

    //from base station
    //compute control mode machine
    
    // Big state machine
    if (control_mode_ == manual) {
        state_machine_manual_callback();

    } else if (control_mode_ == autonomous) {
        state_machine_autonomous_callback();

    } else {
        //Blimp is lost
        forward_command_ = 0.0;
        up_command_ = 0.0;
        yawrate_command_ = 0.0;
    }

    up_motor_ = up_command_; //up for motor
    forward_motor_ = forward_command_; //forward for motor

    //gimbal + motor updates
    ballGrabber.update();

    if (auto_state_ != last_state_) {
        std::string state_str = auto_state_to_string(auto_state_);
        RCLCPP_INFO(this->get_logger(), "State changed to %s", state_str.c_str());
    }

    last_state_ = auto_state_;
}



void CatchingBlimp::state_machine_manual_callback(){
    // publish_log("Im in state_machine_callback, manual");
    //get manual data
    //all motor commands are between -1 and 1
    //set max yaw command to 120 deg/s

    yawrate_command_ = -yaw_msg*120;

    if (USE_EST_VELOCITY_IN_MANUAL) {
        //set max velocities 2 m/s
        up_command_ = up_msg*2.0;
        forward_command_ = forward_msg*2.0;
    } else {
        //normal mapping using max esc command 
        up_command_ = up_msg*750.0; //up is negative
        forward_command_ = forward_msg*750.0;
    }

    //check if shooting should be engaged
    //this block switches the control mode to the oposite that it is currently in
    if (shoot != shootCom) {
        shoot = shootCom;

        if(shoot == 1){
            // Start shooting
            ballGrabber.shoot(control_mode_);

            //reset catch counter
            catches_ = 0;

            //go back to searching
            // auto_state_ = searching;
            // searching timer
            search_start_time_ = state_machine_time_;
            searchYawDirection = searchDirection();  //randomize the search direction

        }else if(shoot == 0){
            // Stop shooting
            ballGrabber.closeGrabber(control_mode_);
        }
    }

    //check if grabbing should be engaged
    //this block switches the control mode to the oposite that it is currently in
    if (grab != grabCom) {
        grab = grabCom;

        if(grab == 1){
            // Start grabbing
            ballGrabber.openGrabber(control_mode_);

            //increase catch counter
            catches_++;

            //start catch timmer
            if (catches_ >= 1) {
                last_catch_time_ = state_machine_time_;
            }

        }else if(grab == 0){
            // Stop grabbing
            ballGrabber.closeGrabber(control_mode_);

        }
    }
}


void CatchingBlimp::state_machine_autonomous_callback(){

    /*---------------------------------------------------------------------------------------------------------------
    ---------------------------Target is determined from the state we are in-----------------------------------------
    ---------------------------------------------------------------------------------------------------------------*/
    
    // Modes for autonomous behavior
    switch (auto_state_) {
        // Autonomous state machine
        case searching:
            state_machine_searching_callback();
            break;
        case approach:
            state_machine_approach_callback();
            break;
        case catching:
            state_machine_catching_callback();
            break;
        case caught:
            state_machine_caught_callback();
            break;
        case goalSearch:
            state_machine_goalSearch_callback();
            break;
        case approachGoal:
            state_machine_approachGoal_callback();
            break;
        case scoringStart:
            state_machine_scoringStart_callback();
            break;
        case shooting:
            state_machine_shooting_callback();
            break;
        case scored:
            state_machine_scored_callback();
            break;
        default: {
            //shouldn't get here
            state_machine_default_callback();
            break;
        }
    } //End auto_mode switch
}


void CatchingBlimp::state_machine_searching_callback(){
    //check if goal scoring should be attempted
    if (catches_ >= 1 && ((state_machine_time_ - last_catch_time_).seconds() >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches_-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
        catches_ = TOTAL_ATTEMPTS;
        auto_state_ = goalSearch;
        goalYawDirection = searchDirection();  //randomize search direction
        return;
    }

    if (catches_ >= TOTAL_ATTEMPTS) {
        auto_state_ = goalSearch;
        goalYawDirection = searchDirection();  //randomize search direction
        return;
    }

    //begin search pattern spinning around at different heights
    if (!(target_detected_ && target_type_ == ball)) {

        //keep ball grabber closed
        ballGrabber.closeGrabber(control_mode_);

        //use object avoidence
        double avoidanceMinVal = 1000.0; // Initialize 
        int avoidanceMinIndex = 10;

        // Iterate through the vector to find the minimum value and its index
        // find the minimum distance and its corresponding quadrant number (1-9)
        // TODO: IMPLEMENT AND TEST AVOIDANCE
        for (int i = 0; i < 9; ++i) {
            if (avoidance[i] < avoidanceMinVal) {
                avoidanceMinVal = avoidance[i]; //distance
                avoidanceMinIndex = i+1; //quadrant number
            }
        }

        //set the avoidance quadrant only when avoidance range is triggered
        if (avoidanceMinVal < AVOID_TRIGGER){
            //update quadrant
            quadrant = avoidanceMinIndex;
        } else {
            //safe
            //update quadrant
            quadrant = 10;
        }

        calculate_avoidance_from_quadrant(quadrant);

        //avoding obstacle
        if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {

            //overide search commands
            forward_command_ = forward_avoidance_;
            up_command_ = up_avoidance_;
            yawrate_command_ = yaw_avoidance_;

        } else {
            //search behavior (no detected_target)
            //spin in a small circle looking for a game ball
            //randomize the search direciton

            // yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            // up_command_ = 0;    //is overriden later, defined here as a safety net
            // forward_command_ = GAME_BALL_FORWARD_SEARCH;
            
            // Timeline:
            // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
            double elapsedSearchingTime = (state_machine_time_ - search_start_time_).seconds();
            std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";

            if (elapsedSearchingTime < TIME_TO_SEARCH) {
                backingUp = false;
                yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                forward_command_ = GAME_BALL_FORWARD_SEARCH;

                if (z_hat_ > CEIL_HEIGHT) {
                    up_command_ = GAME_BALL_VERTICAL_SEARCH; //down
                }else if (z_hat_ < FLOOR_HEIGHT) {
                    up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
                }else{
                    up_command_ = 150;
                }
            } else if (elapsedSearchingTime < TIME_TO_SEARCH + TIME_TO_BACKUP) {
                if (!backingUp) {
                    backingUp = true;
                    searchYawDirection = searchDirection();
                }   
                message += " Backup!";
                
                yawrate_command_ = 25*searchYawDirection;
                forward_command_ = -240;
                up_command_ = 100;
            } else {
                message += " Reset!";
                search_start_time_ = state_machine_time_;
            }

            yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            forward_command_ = GAME_BALL_FORWARD_SEARCH;

            if (z_hat_ > CEIL_HEIGHT) {
                up_command_ = -GAME_BALL_VERTICAL_SEARCH; //down
            } else if (z_hat_ < FLOOR_HEIGHT) {
                up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
            } else {
                up_command_ = 150; //up
            }

            // Prolly hit a net
            // if (20000 < millis()- searchingTimeStart){
            //     double backupTimer = millis();

            //     std::string message = "20+ seconds.";

            //     if (2200 > millis() - backupTimer) {
            //         message += " first 2 seconds.";
            //         searchYawDirection = searchDirection();
            //         yawrate_command_ = 60*searchYawDirection;
            //         up_command_ = 50;    //is overriden later, defined here as a safety net
            //         forward_command_ = -200;
            //     }

            //     searchingTimeStart = millis(); 
            //     publish_log(message.c_str());
            // }

            //move up and down within the set boundry
            // if (z_hat_ > CEIL_HEIGHT) {
            //     // if (wasUp) wasUp = false;
            //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            //     up_command_ = GAME_BALL_VERTICAL_SEARCH; //down
            //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
            // }

            // if (z_hat_ < FLOOR_HEIGHT) {
            //     // if (!wasUp) wasUp = true;
            //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            //     up_command_ = GAME_BALL_VERTICAL_SEARCH;  //up
            //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
            // }

            // if (z_hat_ <= CEIL_HEIGHT && z_hat_ >=FLOOR_HEIGHT) {
            //     // if (wasUp) wasUp = false;
            //     yawrate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            //     up_command_ = 0;
            //     forward_command_ = GAME_BALL_FORWARD_SEARCH;
            // }
        }
    } else {
        //move to approaching game ball
        RCLCPP_INFO(this->get_logger(), "Switched from Search to Approach");
        auto_state_ = approach;

        //start approaching timer
        approach_start_time_ = state_machine_time_;
    }
}


void CatchingBlimp::state_machine_approach_callback(){
    RCLCPP_INFO(this->get_logger(), "Current approach mode: %d at %f meters away (target detected: %s)", approach_state_, target_.z, (target_detected_ ? "true" : "false"));
            
    // Check if maximum time to approach has been exceeded.
    if ((state_machine_time_ - approach_start_time_).seconds() >= MAX_APPROACH_TIME && !BALL_TRACKING_TESTING) {
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;
        // Reset the approach sub–state for the next approach attempt.
        approach_state_ = far_approach;
        return;
    }

    if (target_detected_ && target_type_ == ball) {
        double distance = target_.z;  // Assuming target_.z is the distance measurement.

        switch (approach_state_) {
            case far_approach:
            {
                // When the target is far away, do a direct, head–on approach.
                if (distance > FAR_APPROACH_THRESHOLD) {
                    if(USE_DISTANCE_IN_BALL_APPROACH){
                        float theta_y_target = asin(BASKET_CAMERA_VERTICAL_OFFSET / distance);
                        float theta_y_ball = -target_.theta_y; // target_.theta_y is positive when ball is below center; intentionally flip sign to make it negative
                        RCLCPP_INFO(this->get_logger(), "theta_y_target: %f,  theta_y_ball: %f", theta_y_target, theta_y_ball);

                        up_command_ = -theta_yPID_.calculate(theta_y_target, theta_y_ball, state_machine_dt_);
            
                    }else{
                        up_command_ = yPID_.calculate(GAME_BALL_Y_OFFSET, target_.y, state_machine_dt_);
                    }
                    yawrate_command_ = xPID_.calculate(GAME_BALL_X_OFFSET, target_.x, state_machine_dt_);
                    
                    if(BALL_TRACKING_TESTING){
                        forward_command_= 0;
                    }else{
                        forward_command_= GAME_BALL_CLOSURE_COM;
                    }
                } else {
                    // Once within threshold, switch to alignment mode.
                    approach_state_ = alignment;
                    alignment_start_time_ = state_machine_time_;
                    // Pause forward motion during alignment.
                    forward_command_ = 0;
                    yawrate_command_ = 0;
                    up_command_      = 0;
                }
                break;
            }

            case alignment:
            {
                // In alignment, use the prediction routine (which uses the target history)
                // to estimate where the target is headed and adjust yaw to position the blimp ahead.
                TargetData predicted = predictTargetPosition(ALIGN_PREDICT_HORIZON);
                // Adjust yaw to minimize the alignment error relative to a desired offset (GAME_BALL_X_OFFSET).
                yawrate_command_ = xPID_.calculate(GAME_BALL_X_OFFSET, predicted.x, state_machine_dt_);
                // Optionally hold altitude during alignment.
                if(USE_DISTANCE_IN_BALL_APPROACH){
                    float theta_y_target = asin(BASKET_CAMERA_VERTICAL_OFFSET / distance);
                    float theta_y_ball = -predicted.theta_y; // target_.theta_y is positive when ball is below center; intentionally flip sign to make it negative
                    RCLCPP_INFO(this->get_logger(), "theta_y_target: %f,  theta_y_ball: %f", theta_y_target, theta_y_ball);

                    up_command_ = -theta_yPID_.calculate(theta_y_target, theta_y_ball, state_machine_dt_);
        
                }else{
                    up_command_ = yPID_.calculate(GAME_BALL_Y_OFFSET, predicted.y, state_machine_dt_);
                }
                // Do not command forward motion while aligning.
                forward_command_ = 0;
                
                // After a fixed duration, move to near approach.
                if ((state_machine_time_ - alignment_start_time_).seconds() >= ALIGNMENT_DURATION) {
                    approach_state_ = near_approach;
                }
                break;
            }

            case near_approach:
            {
                // Resume forward approach using similar commands as before.
                double old_yawrate_command_ = xPID_.calculate(GAME_BALL_X_OFFSET, target_.x, state_machine_dt_);
                double x_distance = 0;

                if (target_.z <= 20){
                    last_non_hundred_distance = target_.z;
                    x_distance = std::sin(target_.theta_x) * target_.z;
                    if(target_.z <= 1.0){
                        x_distance *= 1.5;
                    }
                    
                    yawrate_command_ = xPID_.calculate(0, x_distance, state_machine_dt_);
                } else {
                    // use last non 100 distance from array
                    // if(last_non_hundred_distance == 100){
                    //     x_distance = 0; //change to x offset once paramterize setpoint
                    // } else{
                    //     x_distance = last_non_hundred_distance;
                    // }

                    yawrate_command_ = xPID_.calculate(0, 0, state_machine_dt_);
                }

                debug_msg_.data[0] = x_distance;
                debug_msg_.data[1] = yawrate_command_;
                debug_msg_.data[2] = -1* 0.14*target_.x;
                debug_publisher->publish(debug_msg_);


                if(USE_DISTANCE_IN_BALL_APPROACH){
                    float theta_y_target = asin(BASKET_CAMERA_VERTICAL_OFFSET / distance);
                    float theta_y_ball = -target_.theta_y; // target_.theta_y is positive when ball is below center; intentionally flip sign to make it negative
                    RCLCPP_INFO(this->get_logger(), "theta_y_target: %f,  theta_y_ball: %f", theta_y_target, theta_y_ball);

                    up_command_ = -theta_yPID_.calculate(theta_y_target, theta_y_ball, state_machine_dt_);
        
                }else{
                    up_command_ = yPID_.calculate(GAME_BALL_Y_OFFSET, target_.y, state_machine_dt_);
                }

                if(BALL_TRACKING_TESTING){
                    forward_command_= 0;
                    up_command_ = 0;
                }else{
                    forward_command_= GAME_BALL_CLOSURE_COM;
                }

                
                // When very close, transition into the catching state.
                if (distance < BALL_GATE_OPEN_TRIGGER && !BALL_TRACKING_TESTING) {
                    ballGrabber.openGrabber(control_mode_);

                    if (ballGrabber.is_open()) {
                        ballGrabber.suck();
                    }

                    if (distance < BALL_CATCH_TRIGGER && !BALL_TRACKING_TESTING)
                    {   
                        auto_state_ = catching;
                        catch_start_time_ = state_machine_time_;
                        // Reset the sub–state for future approaches.
                        approach_state_ = far_approach;
                    }
                }
                break;
            }
        }
    } else {
        // No target detected: fall back to searching behavior.
        ballGrabber.closeGrabber(control_mode_);
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();
        // Reset the sub–state.
        approach_state_ = far_approach;
    }
}


void CatchingBlimp::state_machine_catching_callback(){
    //Go slower when we get up close
    if (target_.z > 5.0) {
        forward_command_ = CATCHING_FORWARD_COM;
        up_command_ = CATCHING_UP_COM;
    } else {
        forward_command_ = 200;
        up_command_ = CATCHING_UP_COM;
    }

    yawrate_command_ = 0;

    //Turn on the SUCK
    if (ballGrabber.is_open()) {
        ballGrabber.suck();
    }

    if ((state_machine_time_ - catch_start_time_).seconds() >= TIME_TO_CATCH) {
        //catching ended, start caught timer
        auto_state_ = caught;
        caught_start_time_ = state_machine_time_;

        ballGrabber.closeGrabber(control_mode_);

        //increment number of catches_
        catches_ = catches_ + 1;

        //start catch timmer
        last_catch_time_ = state_machine_time_;
    }
}


void CatchingBlimp::state_machine_caught_callback(){
    if (catches_ > 0) {
        //if a target is seen right after the catch
        if (target_detected_ && target_type_ == ball && catches_ < TOTAL_ATTEMPTS) {
            //approach next game ball if visible
            auto_state_ = searching;

            // searching timer
            search_start_time_ = state_machine_time_;
            searchYawDirection = searchDirection();  //randomize the search direction
        }

        //decide if the blimp is going to game ball search or goal search
        if ((state_machine_time_ - caught_start_time_).seconds() >= TIME_TO_CAUGHT) {
            if (catches_ >= TOTAL_ATTEMPTS) {
                auto_state_ = goalSearch;
                goalYawDirection = searchDirection();  //randomize search direction
            } else {
                auto_state_ = searching;
                // searching timer
                search_start_time_ = state_machine_time_;
                searchYawDirection = searchDirection();  //randomize the search direction
            }
        }

        forward_command_ = CAUGHT_FORWARD_COM;
        up_command_ = CAUGHT_UP_COM;
        yawrate_command_ = 0;
    } else {
        auto_state_ = searching;

        // searching timer
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();  //randomize the search direction
    }
}


void CatchingBlimp::state_machine_goalSearch_callback(){
    //keep ball grabber closed
    ballGrabber.closeGrabber(control_mode_);

    //use object avoidence
    double avoidanceMinVal = 1000.0; // Initialize 
    int avoidanceMinIndex = 10;

    // Iterate through the vector to find the minimum value and its index
    // find the minimum distance and its corresponding quadrant number (1-9)
    for (int i = 0; i < 9; ++i) {
        if (avoidance[i] < avoidanceMinVal) {
            avoidanceMinVal = avoidance[i]; //distance
            avoidanceMinIndex = i+1; //quadrant number
        }
    }

    //set the avoidance quadrant only in range
    if (avoidanceMinVal < AVOID_TRIGGER) {
        //update quadrant
        quadrant = avoidanceMinIndex;
    } else {
        //update quadrant
        //safe
        quadrant = 10;
    }

    calculate_avoidance_from_quadrant(quadrant);

    if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {
        //avoiding obstacle
        //override search commands
        forward_command_ = forward_avoidance_;
        up_command_ = up_avoidance_;
        yawrate_command_ = yaw_avoidance_;
    } else {
        //goal search behavior
        //randomize the diretion selection
        yawrate_command_ = GOAL_YAW_SEARCH*goalYawDirection;
        up_command_ = goalPositionHold.calculate(GOAL_HEIGHT, z_hat_);  //go up to the goal
        forward_command_ = GOAL_FORWARD_SEARCH;
    }

    if (target_detected_ && target_type_ == goal) {
        RCLCPP_WARN(this->get_logger(), "DETECTED GOAL - APPROACHING!");
        goal_approach_start_time_ = state_machine_time_;
        auto_state_ = approachGoal;
    }
}


void CatchingBlimp::state_machine_approachGoal_callback(){
    if (target_detected_ && target_type_ == goal) {
        yawrate_command_ = xPID_.calculate(GOAL_X_OFFSET, target_.x, state_machine_dt_);
        up_command_ = yPID_.calculate(GOAL_Y_OFFSET, target_.y, state_machine_dt_);

        if (target_.z > 5) {
            forward_command_ = GOAL_CLOSURE_COM;
        } else {
            forward_command_ = GOAL_CLOSE_COM;
        }

        if (target_.z < GOAL_DISTANCE_TRIGGER) {
            score_start_time_ = state_machine_time_;
            auto_state_ = scoringStart;
        }
    } else {
        // Target memory timeout - start searching for goals again
        auto_state_ = goalSearch;
        goalYawDirection = searchDirection();  //randomize search direction
    }
}


void CatchingBlimp::state_machine_scoringStart_callback(){
    //after correction, we can do goal alignment with a yaw and a translation 
    yawrate_command_ = SCORING_YAW_COM;
    forward_command_ = SCORING_FORWARD_COM;
    up_command_ = SCORING_UP_COM;

    if ((state_machine_time_ - score_start_time_).seconds() >= TIME_TO_SCORE) {
        auto_state_ = shooting;
        shoot_start_time_ = state_machine_time_;
        return;
    }
}


void CatchingBlimp::state_machine_shooting_callback(){
    yawrate_command_ = 0;
    forward_command_ = SHOOTING_FORWARD_COM;
    up_command_ = SHOOTING_UP_COM;

    ballGrabber.shoot(control_mode_);

    if ((state_machine_time_ - shoot_start_time_).seconds() >= TIME_TO_SHOOT) {
        ballGrabber.closeGrabber(control_mode_);
        score_start_time_ = state_machine_time_;
        auto_state_ = scored;
        return;
    }
}


void CatchingBlimp::state_machine_scored_callback(){
    ballGrabber.closeGrabber(control_mode_);

    yawrate_command_ = 0;
    forward_command_ = SCORED_FORWARD_COM;
    up_command_ = SCORED_UP_COM;

    if ((state_machine_time_-score_start_time_).seconds() >= TIME_TO_SCORED) {
        //Reset catch counter and return to ball search state
        catches_ = 0;
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();  //randomize the search direction
    }
}


void CatchingBlimp::state_machine_default_callback(){
    yawrate_command_ = 0;
    forward_command_ = 0;
    up_command_ = 0;
}