#pragma once

#include "rotary_stepper.hpp"

class Homer {
  protected:
    HomingState homing_state;
    int16_t homing_offset = 0;
    Homer():homing_state(HomingState::HOMING_NOT_STARTED) {
    }
  public:
    virtual void loop(RotaryStepper &rotary_stepper) = 0;
    virtual void setup(RotaryStepper &rotary_stepper) = 0;
    HomingState get_homing_state(){
      return homing_state;
    }
    void set_homing_offset(int16_t homing_offset_){
      homing_offset = homing_offset_;
    }
};
