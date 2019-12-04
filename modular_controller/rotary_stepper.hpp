#pragma once

#include <stdint.h>
#include <Arduino.h> 

class RotaryStepper {
  private:
  public:
    uint16_t direction_pin;
    uint16_t step_pin;
    bool dir_high_is_clockwise;
    RotaryStepper(bool dir_high_is_clockwise_, int direction_pin_, int step_pin_):
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_), step_pin(step_pin_) {
    }
    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);

};
