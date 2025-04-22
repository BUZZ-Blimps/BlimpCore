#include "CatchingBlimp.hpp"

void CatchingBlimp::state_machine_callback() {
    update_target();
    
    rclcpp::Time now = this->get_clock()->now();
    state_machine_dt_ = (now - state_machine_time_).seconds();
    state_machine_time_ = now;

    auto debug_msg = std_msgs::msg::Float64MultiArray();

    // from base station
    // compute control mode machine
    
    // Big state machine
    if (control_mode_ == manual) {
        state_machine_manual_callback();
        up_motor_ = up_command_; // up for motor

    } else if (control_mode_ == autonomous) {
        state_machine_autonomous_callback();

        // Constrain z command to physical limits
        if (z_command_ > CEIL_HEIGHT) {
            z_command_ = CEIL_HEIGHT;
        } else if (z_command_ < FLOOR_HEIGHT) {
            z_command_  = FLOOR_HEIGHT;
        }

        
        
    } else {
        //Blimp is lost
        forward_command_ = 0.0;
        up_motor_ = 0.0;
        yaw_rate_command_ = 0.0;
    }

    forward_motor_ = forward_command_; //forward for motor

    // Gimbal + motor updates
    ballGrabber.update();

    if (auto_state_ != last_state_) {
        std::string state_str = auto_state_to_string(auto_state_);
        RCLCPP_INFO(this->get_logger(), "State changed to %s", state_str.c_str());
    }

    last_state_ = auto_state_;
}

void CatchingBlimp::state_machine_manual_callback() {
    // publish_log("Im in state_machine_callback, manual");
    //get manual data
    //all motor commands are between -1 and 1
    //set max yaw command to 120 deg/s

    yaw_rate_command_ = -yaw_rate_msg_*120;

    if (USE_EST_VELOCITY_IN_MANUAL) {
        //set max velocities 2 m/s
        up_command_ = up_msg_*2.0;
        forward_command_ = forward_msg_*2.0;
    } else {
        //normal mapping using max esc command 
        up_command_ = up_msg_*750.0; //up is negative
        forward_command_ = forward_msg_*750.0;
    }

    //check if shooting should be engaged
    //this block switches the control mode to the oposite that it is currently in
    if (shoot != shootCom) {
        shoot = shootCom;

        if(shoot == 1) {
            // Start shooting
            ballGrabber.shoot(control_mode_);

            //reset catch counter
            catches_ = 0;

            //go back to searching
            // auto_state_ = searching;
            // searching timer
            search_start_time_ = state_machine_time_;
            searchYawDirection = searchDirection();  //randomize the search direction

        } else if (shoot == 0) {
            // Stop shooting
            ballGrabber.closeGrabber(control_mode_);
        }
    }

    //check if grabbing should be engaged
    //this block switches the control mode to the oposite that it is currently in
    if (grab != grabCom) {
        grab = grabCom;

        if (grab == 1) {
            // Start grabbing
            ballGrabber.openGrabber(control_mode_);

            //increase catch counter
            catches_++;

            //start catch timmer
            if (catches_ >= 1) {
                last_catch_time_ = state_machine_time_;
            }
        } else if (grab == 0) {
            // Stop grabbing
            ballGrabber.closeGrabber(control_mode_);
        }
    }
}

void CatchingBlimp::state_machine_autonomous_callback() {

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

void CatchingBlimp::state_machine_searching_callback() {
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
    if (!(target_active_ && target_.type == ball)) {

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
            yaw_rate_command_ = yaw_rate_avoidance_;

        } else {
            //search behavior (no detected_target)
            //spin in a small circle looking for a game ball
            //randomize the search direciton

            // yaw_rate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            // forward_command_ = GAME_BALL_FORWARD_SEARCH;
            
            // Timeline:
            // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
            double elapsedSearchingTime = (state_machine_time_ - search_start_time_).seconds();
            std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";

            if (elapsedSearchingTime < TIME_TO_SEARCH) {
                backingUp = false;
                yaw_rate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
                forward_command_ = GAME_BALL_FORWARD_SEARCH;
            } else if (elapsedSearchingTime < TIME_TO_SEARCH + TIME_TO_BACKUP) {
                if (!backingUp) {
                    backingUp = true;
                    searchYawDirection = searchDirection();
                }
                message += " Backup!";
                
                yaw_rate_command_ = 25*searchYawDirection;
                forward_command_ = -240;
                z_command_ += GAME_BALL_VERTICAL_SEARCH*state_machine_dt_;
            } else {
                message += " Reset!";
                search_start_time_ = state_machine_time_;
            }

            yaw_rate_command_ = GAME_BALL_YAW_SEARCH*searchYawDirection;
            forward_command_ = GAME_BALL_FORWARD_SEARCH;

            if (z_hat_ >= CEIL_HEIGHT) {
                z_dir_up_ = false;
            } else if (z_hat_ <= FLOOR_HEIGHT) {
                z_dir_up_  = true;
            }

            if (z_dir_up_) {
                z_command_ += GAME_BALL_VERTICAL_SEARCH*state_machine_dt_;  //up
            } else {
                z_command_ -= GAME_BALL_VERTICAL_SEARCH*state_machine_dt_; //down
            }
        }
    } else {
        //move to approaching game ball
        RCLCPP_INFO(this->get_logger(), "Switched from Search to Approach");
        auto_state_ = approach;

        //start approaching timer
        approach_start_time_ = state_machine_time_;
    }
}

void CatchingBlimp::state_machine_approach_callback() {
    // RCLCPP_INFO(this->get_logger(), "Current approach mode: %d at %f meters away (target detected: %s)", approach_state_, target_.z, (target_active_ ? "true" : "false"));
            
    // Check if maximum time to approach has been exceeded.
    if ((state_machine_time_ - approach_start_time_).seconds() >= MAX_APPROACH_TIME && !BALL_TRACKING_TESTING) {
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;

        return;
    }

    if (target_active_ && target_.type == ball) {

        // Resume forward approach using similar commands as before.
        double x_setpoint = GAME_BALL_X_OFFSET;
        double y_setpoint = 0.0;

        const double bbox_align_min = 900.0;
        const double bbox_align_max = 20000.0;

        if (target_.bbox_area >= bbox_align_min) {
            double scaling = math_helpers::constrain(math_helpers::map(target_.bbox_area, bbox_align_min, bbox_align_max, 1.0, 0.25), 0.25, 1.0);

            // double scaling = math_helpers::constrain(math_helpers::map(target_.bbox_area, 900.0, 20000.0, 1.0, 0.1), 0.1, 1.0);
            xPID_.setPGain(scaling*x_p_);
            xPID_.setDGain(scaling*x_d_);

            // double x_setpoint = math_helpers::constrain(math_helpers::map(target_.bbox_area, bbox_align_min, bbox_align_max, 0.0, GAME_BALL_X_OFFSET), 0.0, GAME_BALL_X_OFFSET);
            double y_setpoint = math_helpers::constrain(math_helpers::map(target_.bbox_area, bbox_align_min, bbox_align_max, 0.0, GAME_BALL_Y_OFFSET), 0.0, GAME_BALL_Y_OFFSET);
        } else {
            xPID_.setPGain(x_p_);
            xPID_.setDGain(x_d_);
        }

        // double y_error = y_setpoint - target_.y;
        // if (y_error > 0) {
        //     // Up go fast so gain normal
        //     yPID_.setPGain(y_p_);
        //     yPID_.setDGain(y_d_);
        // } else {
        //     // Down go slow so gain big
        //     yPID_.setPGain(y_p_);
        //     yPID_.setDGain(2.0*y_d_);
        // }
    
        // Regulate yaw using theta_x
        yaw_rate_command_ = xPID_.calculate(x_setpoint, target_.theta_x, state_machine_dt_);

        // Regulate y using Z-up
        double y_command = yPID_.calculate(y_setpoint, target_.y, state_machine_dt_);
        // RCLCPP_WARN(this->get_logger(), "y_command = %.2f", y_command);

        z_command_ = z_hat_ + y_command*state_machine_dt_;

        if (BALL_TRACKING_TESTING) {
            forward_command_= 0;
        } else {
            forward_command_= GAME_BALL_CLOSURE_COM;
        }

        // When very close, transition into the catching state.
        if (target_.bbox_area >= BALL_GATE_OPEN_TRIGGER && !BALL_TRACKING_TESTING) {
            if (ballGrabber.grabber_state_ != TripleBallGrabber::grabber_state::state_open) {
                RCLCPP_WARN(this->get_logger(), "BALL CLOSE - GRABBER, I BARELY KNOW HER.");

                ballGrabber.openGrabber(control_mode_);
            }

            gate_open_time_ = state_machine_time_;

            if (target_.bbox_area >= BALL_CATCH_TRIGGER && !BALL_TRACKING_TESTING)
            {   
                auto_state_ = catching;
                catch_start_time_ = state_machine_time_;

                // Reset the sub–state for future approaches.
                // approach_state_ = far_approach;
            }
        } else {
            // Wait until ball is out of close range for a certain timeout
            if (ballGrabber.grabber_state_ == TripleBallGrabber::grabber_state::state_open && (state_machine_time_ - gate_open_time_).seconds() >= TIME_TO_OPEN) {
                RCLCPP_WARN(this->get_logger(), "BALL FAR - CLOSING GRABBER.");

                ballGrabber.closeGrabber(control_mode_);
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "BALL LOST - BACK TO SEARCHING.");

        // No target detected: fall back to searching behavior.
        ballGrabber.closeGrabber(control_mode_);
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();

        // Reset the sub–state.
        // approach_state_ = near_approach;
    }
}

void CatchingBlimp::state_machine_catching_callback() {
    //Go slower when we get up close
    if (target_.z > 5.0) {
        forward_command_ = CATCHING_FORWARD_COM;
    } else {
        forward_command_ = 200;
    }

    yaw_rate_command_ = 0;

    // Turn on the SUCK
    if (ballGrabber.is_fully_open() && !ZERO_MODE) {
        ballGrabber.suck();
    }

    if ((state_machine_time_ - catch_start_time_).seconds() >= TIME_TO_CATCH) {
        //catching ended, start caught timer
        auto_state_ = caught;
        caught_start_time_ = state_machine_time_;

        ballGrabber.closeGrabber(control_mode_);

        //increment number of catches_
        RCLCPP_WARN(this->get_logger(), "BALL CAUGHT!");

        catches_++;

        //start catch timmer
        last_catch_time_ = state_machine_time_;
    }
}

void CatchingBlimp::state_machine_caught_callback() {
    if (catches_ > 0) {
        //if a target is seen right after the catch
        if (target_active_ && target_.type == ball && catches_ < TOTAL_ATTEMPTS) {
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
        yaw_rate_command_ = 0;
    } else {
        auto_state_ = searching;

        // searching timer
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();  //randomize the search direction
    }
}

void CatchingBlimp::state_machine_goalSearch_callback() {
    // keep ball grabber closed
    if (ballGrabber.is_open()) {
        ballGrabber.closeGrabber(control_mode_);
    }

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
        // up_command_ = up_avoidance_;
        yaw_rate_command_ = yaw_rate_avoidance_;
    } else {
        //goal search behavior
        //randomize the diretion selection
        yaw_rate_command_ = GOAL_YAW_SEARCH*goalYawDirection;
        // up_command_ = goalPositionHold.calculate(GOAL_HEIGHT, z_hat_);  //go up to the goal
        z_command_ = GOAL_HEIGHT;
        forward_command_ = GOAL_FORWARD_SEARCH;
    }

    if (target_active_ && target_.type == goal) {
        RCLCPP_WARN(this->get_logger(), "DETECTED GOAL - APPROACHING!");
        goal_approach_start_time_ = state_machine_time_;
        auto_state_ = approachGoal;
    }
}

void CatchingBlimp::state_machine_approachGoal_callback(){
    if (target_active_ && target_.type == goal) {
        yaw_rate_command_ = xPID_.calculate(GOAL_X_OFFSET, target_.x, state_machine_dt_);
        double y_command = yPID_.calculate(GOAL_Y_OFFSET, target_.y, state_machine_dt_);

        z_command_ = z_hat_ +  y_command*state_machine_dt_;

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

void CatchingBlimp::state_machine_scoringStart_callback() {
    //after correction, we can do goal alignment with a yaw and a translation 
    yaw_rate_command_ = SCORING_YAW_COM;
    forward_command_ = SCORING_FORWARD_COM;

    if ((state_machine_time_ - score_start_time_).seconds() >= TIME_TO_SCORE) {
        auto_state_ = shooting;
        shoot_start_time_ = state_machine_time_;
        return;
    }
}

void CatchingBlimp::state_machine_shooting_callback() {
    yaw_rate_command_ = 0;
    forward_command_ = SHOOTING_FORWARD_COM;

    ballGrabber.shoot(control_mode_);

    if ((state_machine_time_ - shoot_start_time_).seconds() >= TIME_TO_SHOOT) {
        ballGrabber.closeGrabber(control_mode_);
        score_start_time_ = state_machine_time_;
        auto_state_ = scored;
        return;
    }
}

void CatchingBlimp::state_machine_scored_callback() {
    ballGrabber.closeGrabber(control_mode_);

    yaw_rate_command_ = 0;
    forward_command_ = SCORED_FORWARD_COM;

    if ((state_machine_time_-score_start_time_).seconds() >= TIME_TO_SCORED) {
        //Reset catch counter and return to ball search state
        catches_ = 0;
        auto_state_ = searching;
        search_start_time_ = state_machine_time_;
        searchYawDirection = searchDirection();  //randomize the search direction
    }
}

void CatchingBlimp::state_machine_default_callback() {
    yaw_rate_command_ = 0;
    forward_command_ = 0;
}

