#pragma once

#include "rotary_stepper.hpp"
#include "homing_state.hpp"
#include <Arduino.h>
#include "homer.hpp"

#define TOP_END_STOP_PIN 23
#define BOTTOM_END_STOP_PIN 13

class LinearHomer : public Homer {
  private:
    uint8_t top_end_stop_pin;
    uint8_t bottom_end_stop_pin;
    int32_t total_steps;
    int32_t steps_to_middle;
  public:
    bool rotate_clockwise = false;
    bool top_end_stop_is_on();
    bool bottom_end_stop_is_on();
    void loop(RotaryStepper &rotary_stepper);
    void setup(RotaryStepper &rotary_stepper);
    LinearHomer(): top_end_stop_pin(TOP_END_STOP_PIN),
        bottom_end_stop_pin(BOTTOM_END_STOP_PIN) {
    }
    LinearHomer(uint8_t top_end_stop_pin_,
      uint8_t bottom_end_stop_pin_): top_end_stop_pin(top_end_stop_pin_),
                                             bottom_end_stop_pin(bottom_end_stop_pin_) {

    }
};
