#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include "homing_state.hpp"
#include "homer.hpp"

#define CENTER_MAGNETIC_SENSOR_PIN 13
#define LEFT_MAGNETIC_SENSOR_PIN 14
#define RIGH_MAGNETIC_SENSOR_PIN 12

class RotaryHomer  : public Homer {

  private:
    uint8_t center_magnetic_sensor_pin;
    uint8_t left_magnetic_end_stop_pin;
    uint8_t right_magnetic_end_stop_pin;
    uint16_t steps_with_center_on = 0;
    uint16_t half_steps_backward_left = 0;
    void check_end_stops(RotaryStepper &rotary_stepper);

  public:
    bool rotate_clockwise = true;
    bool center_is_on();
    bool left_is_on();
    bool right_is_on();
    void loop(RotaryStepper &rotary_stepper);
    void setup(RotaryStepper &rotary_stepper);

    RotaryHomer(): center_magnetic_sensor_pin(CENTER_MAGNETIC_SENSOR_PIN),
                                            left_magnetic_end_stop_pin(LEFT_MAGNETIC_SENSOR_PIN),
                                            right_magnetic_end_stop_pin(RIGH_MAGNETIC_SENSOR_PIN),
                                            rotate_clockwise(true) {

    }

};
