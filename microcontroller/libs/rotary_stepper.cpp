#include "rotary_stepper.hpp"

void RotaryStepper::setup() {
  pinMode(this->direction_pin, OUTPUT);
  pinMode(this->step_pin, OUTPUT);
  pinMode(this->enabled_pin, OUTPUT);
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

void RotaryStepper::set_steps(int32_t steps_) {
  steps = steps_;

}
void RotaryStepper::step() {
  digitalWrite(this->step_pin, HIGH);
  delayMicroseconds(50);
  digitalWrite(this->step_pin, LOW);
  delayMicroseconds(50);
  if(direction_pin_state_is_clockwise){
    steps --;
  } else {
    steps ++;
  }
}

void RotaryStepper::enable(bool is_enabled) {
  digitalWrite(this->enabled_pin, !is_enabled);
}

void RotaryStepper::configure(bool dir_high_is_clockwise_, uint16_t direction_pin_, uint16_t step_pin_, ActuatorType actuator_type_) {
  this->dir_high_is_clockwise = dir_high_is_clockwise_;
  this->direction_pin = direction_pin_;
  this->step_pin = step_pin_;
  this->actuator_type = actuator_type_;
  apply_direction(true);
}

uint16_t RotaryStepper::get_step_pin() {
  return step_pin;
}

ActuatorType RotaryStepper::get_actuator_type() {
  return actuator_type;
}

uint16_t RotaryStepper::get_direction_pin() {
  return direction_pin;
}

int32_t RotaryStepper::get_steps() {
  return steps;
}

bool RotaryStepper::get_dir_high_is_clockwise() {
  return this->dir_high_is_clockwise;
}
