#pragma once

#include "rotary_stepper.hpp"
#include "homing_state.hpp"
#include <Arduino.h>
#include "homer.hpp"

class LinearHomer : public Homer {
  private:
    uint8_t top_end_stop_pin;
    uint8_t bottom_end_stop_pin;
    uint32_t total_steps;
    uint32_t steps_to_middle;
  public:
    bool rotate_clockwise = true;
    bool top_end_stop_is_on();
    bool bottom_end_stop_is_on();
    void loop(RotaryStepper &rotary_stepper);
    void setup(RotaryStepper &rotary_stepper);
    LinearHomer(): top_end_stop_pin(0),
                    bottom_end_stop_pin(0){

    }
    LinearHomer(uint8_t top_end_stop_pin_,
      uint8_t bottom_end_stop_pin_): top_end_stop_pin(top_end_stop_pin_),
                                             bottom_end_stop_pin(bottom_end_stop_pin_) {

    }
};
