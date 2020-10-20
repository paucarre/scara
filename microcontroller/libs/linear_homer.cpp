#include "linear_homer.hpp"

bool LinearHomer::top_end_stop_is_on() {
  return digitalRead(top_end_stop_pin);
}

bool LinearHomer::bottom_end_stop_is_on() {
  return digitalRead(bottom_end_stop_pin);
}

void LinearHomer::loop(RotaryStepper &rotary_stepper){
  if(homing_state == HomingState::HOMING_NOT_STARTED){
    rotary_stepper.enable(true);
    rotary_stepper.apply_direction(rotate_clockwise);
    homing_state = HomingState::LINEAR_MOVE_UNITL_BOTTOM_END_STOP;
  }
  if(homing_state != HomingState::HOMING_FINISHED){
    if(homing_state == HomingState::LINEAR_MOVE_UNITL_BOTTOM_END_STOP) {
      int bottom_is_on = bottom_end_stop_is_on();
      if(bottom_is_on){
        homing_state = HomingState::LINEAR_MOVE_UNTIL_TOP_END_STOP;
        rotate_clockwise = ! rotate_clockwise;
        rotary_stepper.apply_direction(rotate_clockwise);
        total_steps = 0;
      }
    } else if(homing_state == HomingState::LINEAR_MOVE_UNTIL_TOP_END_STOP){
      int top_is_on = top_end_stop_is_on();
      if(top_is_on){
        rotate_clockwise = ! rotate_clockwise;
        rotary_stepper.apply_direction(rotate_clockwise);
        homing_state = HomingState::LINEAR_MOVE_TO_HALF;
        steps_to_middle = total_steps / 2;
      } else {
        total_steps ++;
      }
    } else if(homing_state == HomingState::LINEAR_MOVE_TO_HALF){
      if(steps_to_middle <= 0) {
        homing_state = HomingState::HOMING_FINISHED;
      } else {
        steps_to_middle--;
      }
    }
    rotary_stepper.step();
  }
}

void LinearHomer::setup(RotaryStepper &rotary_stepper) {
  pinMode(top_end_stop_pin, INPUT);
  pinMode(bottom_end_stop_pin, INPUT);
  rotary_stepper.apply_direction(rotate_clockwise);
}
