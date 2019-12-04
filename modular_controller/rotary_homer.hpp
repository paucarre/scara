#pragma once

#include <stdint.h>
#include <Arduino.h> 
#include "rotary_stepper.hpp"

enum HomingState {
          MOVE_UNTIL_NO_SENSOR_READ,
          FIND_FIRST_SENSOR_READ,
          READ_UNITIL_SENSOR_NO_LONGER_SENSES,
          REVERSE_DIRECTION_HALF_THE_STEPS,
          HOMING_FINISHED
      };
      
class RotaryHomer {
    
  public:
    uint8_t inductance_sensor_pin;
    HomingState homing_state;
    bool rotate_clockwise = true;
    uint16_t steps_with_zero_on = 0;
    uint16_t half_steps_backward_left = 0;
    bool zero_is_on();
    void loop(RotaryStepper &rotary_stepper);
    void setup(RotaryStepper &rotary_stepper);
    RotaryHomer(uint8_t _inductance_sensor_pin): inductance_sensor_pin(_inductance_sensor_pin),homing_state(HomingState::MOVE_UNTIL_NO_SENSOR_READ){
      
    }
};
