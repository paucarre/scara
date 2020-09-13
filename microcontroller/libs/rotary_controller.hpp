#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"

class RotaryController {
  private:
    int16_t target_steps;
  public:

    RotaryController():
      target_steps(0){
    }

    void set_target_steps(int16_t target_steps_){
      target_steps = target_steps_;
    }

    void control(RotaryStepper& rotary_stepper) {
      // error is 32 bit as difference of 16 bits can be bigger than 16 bits
      int32_t error = rotary_stepper.get_steps() - target_steps;
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
