#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include <math.h>

enum class RotaryControllerState {
    STOPPED,
    ACCELERATING,
    MAX_SPEED,
    DECELERATE
};


class RotaryController {
  private:
    RotaryControllerState controller_state = RotaryControllerState::STOPPED;
    int32_t current_step_to_target = 0;
    int32_t accelerating_steps = 0;
    int32_t maximum_speed_steps = 0;
    int32_t initial_step = 0;

    int32_t target_steps;

    uint16_t accelerating_steps_config;
    uint16_t minimum_speed_config;

    int32_t minimum_steps;
    int32_t maximum_steps;

  public:

    RotaryController():
      target_steps(0), minimum_steps(0), maximum_steps(0) {
        // set_error_constant(5000);
        // set_max_microseconds_delay(500);
    }

    void set_target_steps(int32_t target_steps_){
      target_steps = target_steps_;
      /*
      initial_step = rotary_stepper.get_steps()
      if (controller_state == RotaryControllerState::STOPPED) {
        controller_state = RotaryControllerState::ACCELERATING;
        int32_t diff_steps = abs(rotary_stepper.get_steps() - target_steps);
        if abs(diff_steps < 2 * accelerating_steps) {
          accelerating_steps = diff_steps / 2;
        } else {
          accelerating_steps = accelerating_steps_config;
        }
      } else {
        controller_state = RotaryControllerState.MAX_SPEED;
        int32_t diff_steps = abs(rotary_stepper.get_steps() - target_steps);
        if abs(diff_steps < 2 * accelerating_steps) {
          accelerating_steps = diff_steps / 2;
        } else {
          accelerating_steps = accelerating_steps_config / 2;
        }
      }*/
    }

    int32_t get_minimum_steps(){
      return minimum_steps;
    }

    int32_t get_maximum_steps(){
      return maximum_steps;
    }

    void set_minimum_steps(int32_t minimum_steps_){
      minimum_steps = minimum_steps_;
    }

    void set_maximum_steps(int32_t maximum_steps_){
      maximum_steps = maximum_steps_;
    }

    //uint16_t get_error_constant() {
    //  return error_constant;
    //}

    //uint16_t get_max_microseconds_delay() {
    // return max_microseconds_delay;
    //}

    //void set_error_constant(uint16_t error_constant_) {
    //  error_constant = error_constant_;
    //  error_constant_control =  3.0 / ((float) error_constant);
    //}

    //void set_max_microseconds_delay(uint16_t max_microseconds_delay_) {
    //  max_microseconds_delay = max_microseconds_delay_;
    //  max_microseconds_delay_control = (float) max_microseconds_delay;
    //}

    void control(RotaryStepper& rotary_stepper) {
      // error is 32 bit as difference of 32 bit values that are actually very close to 16 so they won't overflow
      if(maximum_steps > minimum_steps) {
        if(target_steps > maximum_steps){
          //target_steps = maximum_steps;
        } else if(target_steps < minimum_steps) {
          //target_steps = minimum_steps;
        }
      }
      _control(rotary_stepper);
    }

    void _control(RotaryStepper& rotary_stepper) {
      int32_t current_steps = rotary_stepper.get_steps();

      int32_t initial_steps_to_target = abs(initial_step - target_steps);
      int32_t end_of_max_speed_steps = initial_steps_to_target - accelerating_steps;
      int32_t current_steps_to_target = abs(initial_step - target_steps);

      int32_t error = current_steps - target_steps;
      //float absolute_error = abs(error);
      //int microseconds = (int)(max_microseconds_delay_control * exp(-absolute_error * error_constant_control));
      if(error != 0){
        if(controller_state == RotaryControllerState::STOPPED){
          controller_state = RotaryControllerState::ACCELERATING;
        }
        if(current_step_to_target < accelerating_steps) {
          uint32_t delay_in_microseconds = (2. / minimum_speed_config) - ( current_step_to_target / ( minimum_speed_config * accelerating_steps) );
          //delayMicroseconds(delay_in_microseconds);
        } else if (current_step_to_target >= accelerating_steps && current_step_to_target < end_of_max_speed_steps ) {
          uint32_t delay_in_microseconds = 1. / minimum_speed_config;
          //delayMicroseconds(delay_in_microseconds);
        } else if (current_step_to_target >= end_of_max_speed_steps) {
          uint32_t delay_in_microseconds = ( current_step_to_target / ( minimum_speed_config * accelerating_steps) );
          //delayMicroseconds(delay_in_microseconds);
        }
      }
      if(error < 0) {
        // move anticlockwise
        rotary_stepper.apply_direction(false);
        rotary_stepper.step();
      } else if(error > 0) {
        // rotate clockwise
        rotary_stepper.apply_direction(true);
        rotary_stepper.step();
      } else {
        controller_state = RotaryControllerState::STOPPED;
      }
      
    }

};
