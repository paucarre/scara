#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "rotary_stepper.hpp"
#include <math.h>
#include "shared_data.hpp"

enum class RotaryControllerState {
    STOPPED,
    ACCELERATING,
    MAX_SPEED,
    DECELERATE
};
/*
minimum_steps(-100000), maximum_steps(100000), 
max_speed_steps_per_second(1000000), max_acceleration_steps_per_second_squared(20) {
*/

class TrajectoryConfiguration {
  
  private:

    int32_t num_steps_max_acceleration = 0;
    int32_t num_steps_max_speed = 0;
  
  public:

    TrajectoryConfiguration() {
    }
    
    void build(ControlConfigurationData stepper_configuration,  uint32_t total_steps) {
      num_steps_max_acceleration =  ( (float) stepper_configuration.max_speed_steps_per_second ) / 
                                 ( (float) stepper_configuration.max_acceleration_steps_per_second_squared );      
      num_steps_max_speed = total_steps - ( 2 * num_steps_max_acceleration ); // multiply by 2 because there is acceleration and deceleration
      if(num_steps_max_speed < 0) {
        num_steps_max_acceleration = total_steps / 2.0;
        num_steps_max_speed = total_steps - (2 * num_steps_max_acceleration);
      }      
      Serial2.println("TRAJECTORY CONFIG");
      Serial2.println(stepper_configuration.max_speed_steps_per_second);
      Serial2.println(stepper_configuration.max_acceleration_steps_per_second_squared);
      Serial2.println(total_steps);
      Serial2.println(num_steps_max_acceleration);
      Serial2.println(num_steps_max_speed);
    }

    int32_t get_num_steps_max_acceleration() {
      return num_steps_max_acceleration;
    }

    int32_t get_num_steps_max_speed() {
      return num_steps_max_speed;
    }

};

class TrajectoryState {

  private:
    ControlConfigurationData* stepper_configuration;
    TrajectoryConfiguration* trajectory_configuration;

    RotaryControllerState controller_state;
    uint32_t current_step;
    uint32_t total_steps;
    uint32_t current_accelerating_step;
  
  public:

    TrajectoryState() {
    }

    void build(ControlConfigurationData* stepper_configuration_, 
              TrajectoryConfiguration* trajectory_configuration_,
              int32_t total_steps_) {
      stepper_configuration = stepper_configuration_;
      trajectory_configuration = trajectory_configuration_;
      reset();
      total_steps = total_steps_;
    }

    void reset() {
      controller_state = RotaryControllerState::STOPPED;
      current_accelerating_step = 0;
      total_steps = 0; 
      current_step = 0;     
    }

    uint32_t loop(int32_t do_not_use) {
      int32_t speed = 10e5;      
      if(current_step >= total_steps){
        //Serial2.println("STOPPING");
        current_accelerating_step = 0;
        controller_state = RotaryControllerState::STOPPED;
        return 0;
      } else if( current_step < trajectory_configuration->get_num_steps_max_acceleration() ) {        
        //Serial2.println("ACCELERATING");
        current_accelerating_step++;
        controller_state = RotaryControllerState::ACCELERATING;
        speed = stepper_configuration->max_acceleration_steps_per_second_squared * current_accelerating_step;
        current_step++;
      } else if( current_step < trajectory_configuration->get_num_steps_max_acceleration() + 
                trajectory_configuration->get_num_steps_max_speed() )  {
        //Serial2.println("MAX_SPEED");
        if(controller_state != RotaryControllerState::MAX_SPEED) {
          controller_state = RotaryControllerState::MAX_SPEED;
          current_accelerating_step = 0;
        }
        speed = stepper_configuration->max_speed_steps_per_second;
        current_step++;
      } else {
        if(controller_state != RotaryControllerState::DECELERATE) {
          current_accelerating_step = 0;
          controller_state = RotaryControllerState::DECELERATE;
        }
        current_accelerating_step++;        
        //Serial2.println("DECELERATE");
        int32_t test = trajectory_configuration->get_num_steps_max_acceleration() - current_accelerating_step;
        speed = stepper_configuration->max_acceleration_steps_per_second_squared * test;
        if(speed <= 0) {
          speed = 10e5;
        }
        current_step++;
      }
      uint32_t delay = (uint32_t)(10e5 / ((float)speed));
      return delay;
    }

    RotaryControllerState get_controller_state(){
      return controller_state;
    }

    uint32_t get_current_step() {
      return current_step;
    }
    
    uint32_t get_total_steps() {
      return total_steps;
    }
    
    uint32_t get_current_accelerating_step() {
      return current_accelerating_step;
    }
  
};

class RotaryController {

  private:
  
    ControlConfigurationData stepper_configuration;
    TrajectoryConfiguration trajectory_configuration;
    TrajectoryState trajectory_state;
    
    int32_t target_steps  = 0;
    int32_t current_steps = 0;

  public:

    RotaryController():
      target_steps(0) {      
      trajectory_configuration.build(stepper_configuration,  0);
      trajectory_state.build(&stepper_configuration, &trajectory_configuration, 0);
    }
    
    void set_target_steps(int32_t target_steps_){
      if(target_steps_ != target_steps) {
        target_steps = target_steps_;      
        int32_t total_steps = target_steps - current_steps;
        if(total_steps < 0) {
          total_steps = - total_steps;
        }
        trajectory_configuration.build(stepper_configuration,  total_steps);
        trajectory_state.build(&stepper_configuration, &trajectory_configuration, total_steps);        
      }
    }

    ControlConfigurationData get_control_configuration_data(){
      return stepper_configuration;
    }

    void set_control_configuration_data(ControlConfigurationData control_configuration_data_){
      stepper_configuration = control_configuration_data_;
    }

    void control(RotaryStepper& rotary_stepper) {
      // error is 32 bit as difference of 32 bit values that are actually very close to 16 so they won't overflow
      int32_t maximum_steps = stepper_configuration.maximum_steps;
      int32_t minimum_steps = stepper_configuration.minimum_steps;
      if(maximum_steps > minimum_steps) {
        if(target_steps > maximum_steps){
          target_steps = maximum_steps;
        } else if(target_steps < minimum_steps) {
          target_steps = minimum_steps;
        }
      }
      _control(rotary_stepper);
    }

    void _control(RotaryStepper& rotary_stepper) {
      
      current_steps  = rotary_stepper.get_steps();
      int32_t error  = current_steps - target_steps;
      //Serial2.println("loop call");
      uint32_t delay = trajectory_state.loop(current_steps);      
      if(trajectory_state.get_controller_state() != RotaryControllerState::STOPPED){        
        if(delay > 0) {
          delayMicroseconds(delay);
        }
        if(error < 0) {
          // rotate anticlockwise
          rotary_stepper.apply_direction(false);
          rotary_stepper.step();
        } else if(error > 0) {
          // rotate clockwise
          rotary_stepper.apply_direction(true);
          rotary_stepper.step();
        }
      }// else {
        //Serial2.println("STOPPED AND THUS DOING NOTHING");
      //}
    }

};
