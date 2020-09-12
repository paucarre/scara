#include "rotary_stepper.hpp"

void RotaryStepper::setup() {
  pinMode(this->direction_pin, OUTPUT);
  pinMode(this->step_pin, OUTPUT);
  digitalWrite(this->step_pin, HIGH);
  this->apply_direction(true);
}

void RotaryStepper::apply_direction(bool rotate_clockwise) {
  direction_pin_state_is_clockwise = rotate_clockwise;
  //Serial.println( "rotate_clockwise:" + String(rotate_clockwise) + " --- is_clockwise : " + String(this->dir_high_is_clockwise) );
  if( (this->dir_high_is_clockwise && direction_pin_state_is_clockwise) || ((!direction_pin_state_is_clockwise) && (!this->dir_high_is_clockwise)) ) {
    //Serial.println("rotary_stepper - Setting direction to High");
    digitalWrite(this->direction_pin, HIGH);
  } else {
    //Serial.println("rotary_stepper - Setting direction to LOW");
    digitalWrite(this->direction_pin, LOW);
  }
}

void RotaryStepper::set_steps_to_zero() {
  steps = 0;
}

void RotaryStepper::step() {
  digitalWrite(this->step_pin, HIGH);
  delayMicroseconds(50);
  digitalWrite(this->step_pin, LOW);
  delayMicroseconds(50);
  if(direction_pin_state_is_clockwise){
    steps ++;
  } else {
    steps --;
  }
}
