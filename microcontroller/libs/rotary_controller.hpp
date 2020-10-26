#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include <math.h>

class RotaryController {
  private:
    int32_t target_steps;
    float time_constant = 3.0 / 5000.0;
    float max_microseconds_delay = 500.0;

  public:

    RotaryController():
      target_steps(0){
    }

    void set_target_steps(int32_t target_steps_){
      target_steps = target_steps_;
    }

    void control(RotaryStepper& rotary_stepper) {
      // error is 32 bit as difference of 32 bit values that are actually very close to 16 so they won't overflow
      int32_t error = rotary_stepper.get_steps() - target_steps;
      float absolute_error = abs(error);
      int microseconds = (int)(max_microseconds_delay * exp(-absolute_error * time_constant));
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
