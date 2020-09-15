#include "rotary_homer.hpp"

bool RotaryHomer::center_is_on() {
    int attempts_strong_signal = 1000;
    bool is_on = true;
    // Veryfy the signal is strong
    for(int attempts = 0; is_on && attempts < attempts_strong_signal;attempts++) {
      is_on = digitalRead(center_magnetic_sensor_pin);
    }
    return is_on;
}

bool RotaryHomer::left_is_on() {
  return digitalRead(left_magnetic_end_stop_pin);
}

bool RotaryHomer::right_is_on() {
  return digitalRead(right_magnetic_end_stop_pin);
}

void RotaryHomer::check_end_stops(RotaryStepper &rotary_stepper){
  int right_on = right_is_on();
  int left_on = left_is_on();
  if(right_on || left_on) {
    rotate_clockwise = !rotate_clockwise;

    rotary_stepper.apply_direction(rotate_clockwise);
    for(uint8_t i = 0; (left_is_on() || right_is_on()) && i < 500; i ++){
        rotary_stepper.step();
        delayMicroseconds(3000);
    }
  }
}

void RotaryHomer::loop(RotaryStepper &rotary_stepper){
  if(homing_state == HomingState::HOMING_NOT_STARTED){
    rotary_stepper.apply_direction(rotate_clockwise);
    homing_state = HomingState::MOVE_UNTIL_NO_SENSOR_READ;
  }
  if(homing_state != HomingState::HOMING_FINISHED){
    int center_on = center_is_on();
    check_end_stops(rotary_stepper);
    //Serial.println("rotary_homer - " + String(homing_state) + " | " + String(center_on));
    if(center_on) {
      if(homing_state == HomingState::FIND_FIRST_SENSOR_READ) {
        homing_state = HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES;
      }
      if(homing_state == HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES ){
        steps_with_center_on ++;
      }
      if(homing_state == HomingState::REVERSE_DIRECTION_HALF_THE_STEPS){
        if(half_steps_backward_left > 0){
          half_steps_backward_left --;
        } else {
          homing_state = HomingState::HOMING_FINISHED;
          rotary_stepper.set_steps_to_zero();
        }
      }
    } else {
      if(homing_state == HomingState::MOVE_UNTIL_NO_SENSOR_READ) {
        rotate_clockwise = ! rotate_clockwise;
        rotary_stepper.apply_direction(rotate_clockwise);
        homing_state = HomingState::FIND_FIRST_SENSOR_READ;
      } else if(homing_state == HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES){
        rotate_clockwise = ! rotate_clockwise;
        rotary_stepper.apply_direction(rotate_clockwise);
        homing_state = HomingState::REVERSE_DIRECTION_HALF_THE_STEPS;
        half_steps_backward_left = steps_with_center_on / 2;
      }
    }
    //Serial.println("rotary_homer - stepping");
    rotary_stepper.step();
  }
}

void RotaryHomer::setup(RotaryStepper &rotary_stepper) {
  pinMode(center_magnetic_sensor_pin, INPUT);
  pinMode(left_magnetic_end_stop_pin  , INPUT);
  pinMode(right_magnetic_end_stop_pin , INPUT);
  rotary_stepper.apply_direction(rotate_clockwise);
}
