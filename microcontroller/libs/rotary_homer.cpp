#include "rotary_homer.hpp"

bool RotaryHomer::center_is_on() {
  return digitalRead(center_magnetic_sensor_pin);
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
    if(right_on) {
      // one can arrive to the right end stop by going clockwise, if that happens going
      // counterclockwise, then the rotary stepper is not properly configured.
      //Serial.println("Right magnetic end stop sensor found");
      if(!rotary_stepper.direction_pin_state_is_clockwise){
        //Serial.println("Moving anticlockwise and reaching left magnetic sensor");
        rotary_stepper.dir_high_is_clockwise = !rotary_stepper.dir_high_is_clockwise;
        rotary_stepper.direction_pin_state_is_clockwise = true;
      }
      // regardless, change rotation to get away from end stop
      rotate_clockwise = false;
      rotary_stepper.apply_direction(rotate_clockwise);
      for(uint8_t i = 0; right_is_on() && i < 100; i ++){      
        rotary_stepper.step();
        delay(1);
      }
    } else {
      //Serial.println("Left magnetic end stop sensor found");
      // one can arrive to the left end stop by going anticlockwise, if that happens going
      // clockwise, then the rotary stepper is not properly configured.
      if(rotary_stepper.direction_pin_state_is_clockwise){
        //Serial.println("Moving clockwise and reaching left magnetic sensor");
        rotary_stepper.dir_high_is_clockwise = !rotary_stepper.dir_high_is_clockwise;
        rotary_stepper.direction_pin_state_is_clockwise = false;
      }
      // regardless, change rotation to get away from end stop
      rotate_clockwise = true;
      rotary_stepper.apply_direction(rotate_clockwise);
      for(uint8_t i = 0; left_is_on() && i < 100; i ++){      
        rotary_stepper.step();
        delay(1);
      }
    }
  }
}

void RotaryHomer::loop(RotaryStepper &rotary_stepper) {
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
      half_steps_backward_left = steps_with_center_on / 2;
    }
  }
  if(homing_state != HomingState::HOMING_FINISHED){
    rotary_stepper.step();
  }
}

void RotaryHomer::setup(RotaryStepper &rotary_stepper) {
  pinMode(center_magnetic_sensor_pin, INPUT);
  pinMode(left_magnetic_end_stop_pin  , INPUT);
  pinMode(right_magnetic_end_stop_pin , INPUT);
  rotary_stepper.apply_direction(rotate_clockwise);
}
