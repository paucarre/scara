#include "rotary_homer.hpp"

bool RotaryHomer::zero_is_on() {
  return !digitalRead(inductance_sensor_pin);
}

void RotaryHomer::loop(RotaryStepper &rotary_stepper) {
  int zero_on = zero_is_on();
  //Serial.println("rotary_homer - " + String(homing_state));
  //Serial.println("rotary_homer - " + String(zero_on));
  if(zero_on) {
    if(homing_state == HomingState::FIND_FIRST_SENSOR_READ) {
      homing_state = HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES;
    }    
    if(homing_state == HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES ){
      steps_with_zero_on ++;
    }
    if(homing_state == HomingState::REVERSE_DIRECTION_HALF_THE_STEPS){
      if(half_steps_backward_left > 0){
        half_steps_backward_left --;
      } else {
        homing_state = HomingState::HOMING_FINISHED;
      }
    }
  } else {
    if(homing_state == MOVE_UNTIL_NO_SENSOR_READ) {
      rotate_clockwise = ! rotate_clockwise;
      rotary_stepper.apply_direction(rotate_clockwise);
      homing_state = HomingState::FIND_FIRST_SENSOR_READ;
    } else if(homing_state == HomingState::READ_UNITIL_SENSOR_NO_LONGER_SENSES){
      rotate_clockwise = ! rotate_clockwise;
      rotary_stepper.apply_direction(rotate_clockwise);
      homing_state = HomingState::REVERSE_DIRECTION_HALF_THE_STEPS;
      half_steps_backward_left = steps_with_zero_on / 2;
    }
  }
  if(homing_state != HomingState::HOMING_FINISHED){
    rotary_stepper.step();
  }
}

void RotaryHomer::setup(RotaryStepper &rotary_stepper) {
  pinMode(inductance_sensor_pin, INPUT);
  rotary_stepper.apply_direction(rotate_clockwise);
}
