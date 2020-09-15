#pragma once

#include <stdint.h>
#include <Arduino.h>

class RotaryStepper {
  private:
    bool direction_pin_state_is_clockwise;
    uint16_t direction_pin;
    uint16_t step_pin;
    int16_t steps;
    bool dir_high_is_clockwise;
  public:

    RotaryStepper(bool dir_high_is_clockwise_, uint16_t direction_pin_, uint16_t step_pin_):
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_), step_pin(step_pin_), steps(0) {
        // by default always rotate clockwise if no direction is provided
        apply_direction(true);
    }

    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);
    void set_steps_to_zero();

    void configure(bool dir_high_is_clockwise_, uint16_t direction_pin_, uint16_t step_pin_);

    uint16_t get_step_pin();

    uint16_t get_direction_pin();

    int16_t get_steps();

    bool get_dir_high_is_clockwise();
};
