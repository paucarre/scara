#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include <math.h>

class RotaryController {
  private:
    int32_t target_steps;

    uint16_t error_constant;
    uint16_t max_microseconds_delay;

    float error_constant_control;
    float max_microseconds_delay_control;

  public:

    RotaryController():
      target_steps(0){
        set_error_constant(5000);
        set_max_microseconds_delay(500);
    }

    void set_target_steps(int32_t target_steps_){
      target_steps = target_steps_;
    }

    uint16_t get_error_constant() {
      return error_constant;
    }

    uint16_t get_max_microseconds_delay() {
      return max_microseconds_delay;
    }

    void set_error_constant(uint16_t error_constant_) {
      error_constant = error_constant_;
      error_constant_control =  3.0 / ((float) error_constant);
    }

    void set_max_microseconds_delay(uint16_t max_microseconds_delay_) {
      max_microseconds_delay = max_microseconds_delay_;
      max_microseconds_delay_control = (float) max_microseconds_delay;
    }

    void control(RotaryStepper& rotary_stepper) {
      // error is 32 bit as difference of 32 bit values that are actually very close to 16 so they won't overflow
      int32_t error = rotary_stepper.get_steps() - target_steps;
      float absolute_error = abs(error);
      int microseconds = (int)(max_microseconds_delay_control * exp(-absolute_error * error_constant_control));
      delayMicroseconds(microseconds);
      if(error < 0) {
        // move anticlockwise
        rotary_stepper.apply_direction(false);
        rotary_stepper.step();
      } else if(error > 0) {
        // rotate clockwise
        rotary_stepper.apply_direction(true);
        rotary_stepper.step();
      }
    }

};
