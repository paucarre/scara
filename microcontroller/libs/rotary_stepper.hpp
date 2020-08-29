#pragma once

#include <stdint.h>
#include <Arduino.h> 

class RotaryStepper {
  private:
  public:
    uint16_t direction_pin;
    uint16_t step_pin;
    int32_t current_step;
    bool dir_high_is_clockwise;
    bool direction_pin_state_is_clockwise;
    RotaryStepper(bool dir_high_is_clockwise_, int direction_pin_, int step_pin_):
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_), step_pin(step_pin_), current_step(0), direction_pin_state_is_clockwise(dir_high_is_clockwise_) {                 
    }
    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);
    void set_current_step_as_zero();

};
