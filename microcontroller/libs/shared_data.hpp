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
  uint16_t error_constant = 0;
  uint16_t max_microseconds_delay = 0;
  int32_t minimum_steps = 0;
  int32_t maximum_steps = 0;
};

struct SharedData {
  ActionsData  actions;
  HomingState homing_state = HomingState::HOMING_NOT_STARTED;
  ConfigurationData configuration;
  ControlData control;
  ControlConfigurationData control_configuration_data;
};
