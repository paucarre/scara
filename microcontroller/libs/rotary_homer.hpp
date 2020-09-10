#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include "homing_states.hpp"

class RotaryHomer {
  private:
    uint8_t center_magnetic_sensor_pin;
    uint8_t left_magnetic_end_stop_pin;
    uint8_t right_magnetic_end_stop_pin;
    uint16_t steps_with_center_on = 0;
    uint16_t half_steps_backward_left = 0;
    void check_end_stops(RotaryStepper &rotary_stepper);
  public:
    HomingState homing_state;
    bool rotate_clockwise = true;
    bool center_is_on();
    bool left_is_on();
    bool right_is_on();
    void loop(RotaryStepper &rotary_stepper);
    void setup(RotaryStepper &rotary_stepper);
    RotaryHomer(uint8_t _center_magnetic_sensor_pin,
      uint8_t _left_magnetic_end_stop_pin,
      uint8_t _right_magnetic_end_stop_pin): center_magnetic_sensor_pin(_center_magnetic_sensor_pin),
                                             left_magnetic_end_stop_pin(_left_magnetic_end_stop_pin),
                                             right_magnetic_end_stop_pin(_right_magnetic_end_stop_pin),
                                             homing_state(HomingState::HOMING_NOT_STARTED){

    }
};
