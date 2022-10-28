#include "homing_state.hpp"
#include "homing_state.hpp"
#pragma once

struct ConfigurationData {
  bool dir_high_is_clockwise = false;
  uint8_t direction_pin = 27;
  uint8_t step_pin = 26;
  ActuatorType actuator_type = ActuatorType::ROTARY;
  int16_t homing_offset = 0;
};

struct ActionsData {
  bool do_homing = false;
  bool do_configure = false;
};

struct ControlData {
  int32_t steps = 0;
  int32_t target_steps = 0;
};

struct ControlConfigurationData {
  int32_t minimum_steps;
  int32_t maximum_steps;
  uint32_t max_speed_steps_per_second;
  uint32_t max_acceleration_steps_per_second_squared;
};

struct ServoControlData {
  uint32_t duty;
};

struct SharedData {
  ActionsData  actions;
  HomingState homing_state = HomingState::HOMING_FINISHED;
  ConfigurationData configuration;
  ControlData control;
  ControlConfigurationData control_configuration_data;
  ServoControlData servo_control_data;
};
