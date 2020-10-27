#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "actuator_type.hpp"

class RotaryStepper {
  private:
    ActuatorType actuator_type;
    bool direction_pin_state_is_clockwise;
    uint16_t direction_pin;
    uint16_t step_pin;
    uint16_t enabled_pin;
    int32_t steps;
    bool dir_high_is_clockwise;
  public:

    RotaryStepper(bool dir_high_is_clockwise_, uint16_t direction_pin_,
      uint16_t step_pin_, uint16_t enabled_pin_, ActuatorType actuator_type_):
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_),
      step_pin(step_pin_), enabled_pin(enabled_pin_), actuator_type(actuator_type_), steps(0) {
        // by default always rotate clockwise if no direction is provided
        apply_direction(true);
        enable(false);
    }

    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);
    void set_steps(int32_t steps);
    void enable(bool is_enabled);

    void configure(bool dir_high_is_clockwise_, uint16_t direction_pin_, uint16_t step_pin_, ActuatorType actuator_type_);

    uint16_t get_step_pin();

    ActuatorType get_actuator_type();

    uint16_t get_direction_pin();

    int32_t get_steps();

    bool get_dir_high_is_clockwise();
};
