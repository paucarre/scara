#pragma once

#include <stdint.h>
#include <Arduino.h>

class RotaryStepper {
  private:
    uint16_t direction_pin;
    uint16_t step_pin;
    int16_t steps;
    bool dir_high_is_clockwise;
    bool direction_pin_state_is_clockwise;
  public:

    RotaryStepper(bool dir_high_is_clockwise_, int direction_pin_, int step_pin_):
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_), step_pin(step_pin_), steps(0), direction_pin_state_is_clockwise(dir_high_is_clockwise_) {
    }
    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);
    void set_steps_to_zero();

    uint8_t get_step_pin() {
      return step_pin;
    }

    uint8_t get_direction_pin() {
      return direction_pin;
    }

    int16_t get_steps() {
      return steps;
    }

    uint8_t get_dir_high_is_clockwise() {
      return dir_high_is_clockwise;
    }

    void configure(bool dir_high_is_clockwise_, uint16_t direction_pin_, uint16_t step_pin_) {
      direction_pin_state_is_clockwise = dir_high_is_clockwise_;
      direction_pin = direction_pin_;
      step_pin = step_pin_;
    }

};
