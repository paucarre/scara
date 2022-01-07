#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"
#include "linear_homer.hpp"
#include "rotary_controller.hpp"
#include "shared_data.hpp"
#include "protocol.hpp"
#include "homer_builder.hpp"


#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

SharedData shared_data;
SemaphoreHandle_t mutex;
protocol::Parser parser;

template<typename F>
void do_safely_sharing_data(F &lambda) {
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {
      lambda();
      xSemaphoreGive(mutex);
    }
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(communication, "communication", 10000, NULL, 1, NULL,  1);
  delay(1000);
  xTaskCreatePinnedToCore(control, "control", 10000, NULL, 2, NULL,  0);
  delay(1000);
}

void loop() {
}

void write_message(protocol::Message message) {
  Serial.write((const uint8_t *)message.message, (int)message.get_message_length());
  Serial.flush();
}


void control( void * pvParameters ) {
  Homer* homer = &HomerBuilder::build_homer(ActuatorType::ROTARY);
  SharedData controller_data;
  RotaryStepper rotary_stepper(false, 27, 26, 25, ActuatorType::ROTARY);
  rotary_stepper.setup();
  bool homer_is_initialized = false;
  RotaryController rotary_controller;
  auto init_configuration = [&shared_data, &rotary_stepper, &rotary_controller] () {
      shared_data.configuration.dir_high_is_clockwise = rotary_stepper.get_dir_high_is_clockwise();
      shared_data.configuration.direction_pin = rotary_stepper.get_direction_pin();
      shared_data.configuration.step_pin = rotary_stepper.get_step_pin();
      shared_data.configuration.actuator_type = rotary_stepper.get_actuator_type();
      shared_data.actions.do_configure = false;
      //TODO: remove this or support it again
      shared_data.control_configuration_data.error_constant = 0.0; //rotary_controller.get_error_constant();
      shared_data.control_configuration_data.max_microseconds_delay = 0.0; //rotary_controller.get_max_microseconds_delay();
      shared_data.control_configuration_data.minimum_steps = rotary_controller.get_minimum_steps();
      shared_data.control_configuration_data.maximum_steps = rotary_controller.get_maximum_steps();
  };
  do_safely_sharing_data(init_configuration);

  for (;;) {
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    auto pull_protocol_configuration = [&] () { controller_data = shared_data; };
    do_safely_sharing_data(pull_protocol_configuration);
    //TODO: remove this or support it again
    //rotary_controller.set_error_constant(controller_data.control_configuration_data.error_constant);
    //rotary_controller.set_max_microseconds_delay(controller_data.control_configuration_data.max_microseconds_delay);
    rotary_controller.set_minimum_steps(controller_data.control_configuration_data.minimum_steps);
    rotary_controller.set_maximum_steps(controller_data.control_configuration_data.maximum_steps);
    if (controller_data.actions.do_homing) {
      if(!homer_is_initialized){
        homer = &HomerBuilder::build_homer(rotary_stepper.get_actuator_type());
        homer->setup(rotary_stepper);
        homer_is_initialized = true;
      }
      homer->loop(rotary_stepper);
      controller_data.homing_state = homer->get_homing_state();
      if(controller_data.homing_state == HomingState::HOMING_FINISHED){
        controller_data.actions.do_homing = false;
        homer_is_initialized = false;
      }
      auto update_homing_state = [&shared_data, &controller_data] () { 
        shared_data.homing_state = controller_data.homing_state;
        shared_data.actions.do_homing = controller_data.actions.do_homing; 
      };
      do_safely_sharing_data(update_homing_state);
    }
    if (controller_data.actions.do_configure) {
      rotary_stepper.configure(
        controller_data.configuration.dir_high_is_clockwise,
        controller_data.configuration.direction_pin,
        controller_data.configuration.step_pin,
        controller_data.configuration.actuator_type);
      homer->set_homing_offset(controller_data.configuration.homing_offset);
      auto configuration_finished = [&shared_data, &rotary_stepper] () {
        shared_data.configuration.dir_high_is_clockwise = rotary_stepper.get_dir_high_is_clockwise();
        shared_data.configuration.direction_pin = rotary_stepper.get_direction_pin();
        shared_data.configuration.step_pin = rotary_stepper.get_step_pin();
        shared_data.configuration.actuator_type = rotary_stepper.get_actuator_type();
        shared_data.actions.do_configure = false;
        };
      do_safely_sharing_data(configuration_finished);
    }
    int32_t steps = rotary_stepper.get_steps();
    auto update_steps = [&shared_data, &steps] () { shared_data.control.steps = steps; };
    do_safely_sharing_data(update_steps);
    //if(controller_data.homing_state == HomingState::HOMING_FINISHED) {
    rotary_controller.set_target_steps(controller_data.control.target_steps);
    //Serial2.println("Control Loop - control call");
    rotary_controller.control(rotary_stepper);
    //}
  }
}


protocol::ParsingResult parsing_result;

void communication( void * pvParameters ) {
  for (;;) {
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
    if (Serial.available() > 0) {
      char received_byte = Serial.read();
      parsing_result = parser.parse_byte(received_byte);
      if (parsing_result.get_is_parsed()) {
        protocol::Message message = parsing_result.get_message();
        if (message.get_message_type() == protocol::HOME_MESSAGE_TYPE) {
          auto activate_homing = [&shared_data] () { shared_data.actions.do_homing = true; };
          do_safely_sharing_data(activate_homing);
          protocol::Message message_return = protocol::Message::make_homing_response_message();
          write_message(message_return);
        } else if(message.get_message_type() == protocol::CONFIGURE_MESSAGE_TYPE){
          bool dir_high_is_clockwise = message.data[0];
          uint8_t direction_pin = message.data[1];
          uint8_t step_pin = message.data[2];
          int16_t homing_offset = protocol::Message::make_int16_from_two_bytes(message.data[3], message.data[4]);
          ActuatorType actuator_type = static_cast<ActuatorType>(message.data[5]);
          auto copy_configuration = [&, dir_high_is_clockwise, direction_pin, step_pin, homing_offset, actuator_type] () {
            shared_data.configuration.dir_high_is_clockwise = dir_high_is_clockwise;
            shared_data.configuration.direction_pin = direction_pin;
            shared_data.configuration.step_pin = step_pin;
            shared_data.configuration.homing_offset = homing_offset;
            shared_data.configuration.actuator_type = actuator_type;
            shared_data.actions.do_configure = true;
          };
          do_safely_sharing_data(copy_configuration);
          protocol::Message message_return = protocol::Message::make_configure_response_message(dir_high_is_clockwise, direction_pin, step_pin, homing_offset, actuator_type);
          write_message(message_return);
        } else if(message.get_message_type() == protocol::HOMING_STATE_MESSAGE_TYPE){
          int32_t homing_state = static_cast<int>(HomingState::LINEAR_MOVE_UNTIL_TOP_END_STOP);
          auto get_homing_state = [&homing_state, &shared_data] () { homing_state = static_cast<int>(shared_data.homing_state); };
          do_safely_sharing_data(get_homing_state);
          protocol::Message message_return = protocol::Message::make_homing_state_response_message((char)(0x000F & homing_state));
          write_message(message_return);
        } else if(message.get_message_type() == protocol::GET_STEPS_MESSAGE_TYPE) {
          int32_t steps = 0;
          auto update_steps = [&shared_data, &steps] () { steps = shared_data.control.steps; };
          do_safely_sharing_data(update_steps);
          protocol::Message message_return = protocol::Message::make_get_steps_response_message(steps);
          write_message(message_return);
        } else if(message.get_message_type() == protocol::SET_TARGET_STEPS_MESSAGE_TYPE) {
          int32_t target_steps = protocol::Message::make_int32_from_four_bytes(message.data[0], message.data[1], message.data[2], message.data[3]);
          auto update_target_steps = [&shared_data, &target_steps] () { shared_data.control.target_steps = target_steps; };
          do_safely_sharing_data(update_target_steps);
          protocol::Message message_return = protocol::Message::make_set_target_steps_response_message(target_steps);
          write_message(message_return);
        } else if(message.get_message_type() == protocol::GET_TARGET_STEPS_MESSAGE_TYPE) {
          int32_t target_steps = 0;
          auto update_target_steps = [&shared_data, &target_steps] () { target_steps = shared_data.control.target_steps; };
          do_safely_sharing_data(update_target_steps);
          protocol::Message message_return = protocol::Message::make_get_target_steps_response_message(target_steps);
          write_message(message_return);
        } else if(message.get_message_type() == protocol::GET_CONFIGURATION_MESSAGE_TYPE) {
          char dir_high_is_clockwise = 0;
          char direction_pin = 0;
          char step_pin = 0;
          int16_t homing_offset = 0;
          ActuatorType actuator_type = ActuatorType::ROTARY;
          auto configuration = [&shared_data, &dir_high_is_clockwise, &direction_pin, &step_pin, &homing_offset, &actuator_type] () {
            dir_high_is_clockwise = shared_data.configuration.dir_high_is_clockwise;
            direction_pin = shared_data.configuration.direction_pin;
            step_pin = shared_data.configuration.step_pin;
            homing_offset = shared_data.configuration.homing_offset;
            actuator_type = shared_data.configuration.actuator_type;
          };
          do_safely_sharing_data(configuration);
          protocol::Message message_return = protocol::Message::make_get_configuration_response_message(dir_high_is_clockwise, direction_pin, step_pin, homing_offset, actuator_type);
          write_message(message_return);
        } else if (message.get_message_type() == protocol::SET_CONTROL_CONFIGURATION_MESSAGE_TYPE) {
          uint16_t error_constant = protocol::Message::make_uint16_from_two_bytes(message.data[0], message.data[1]);
          uint16_t max_microseconds_delay = protocol::Message::make_uint16_from_two_bytes(message.data[2], message.data[3]);
          auto update_control_config = [&shared_data, &error_constant, &max_microseconds_delay] () {
            shared_data.control_configuration_data.error_constant = error_constant;
            shared_data.control_configuration_data.max_microseconds_delay = max_microseconds_delay;
          };
          do_safely_sharing_data(update_control_config);
          protocol::Message message_return = protocol::Message::make_set_control_configuration_response_message(error_constant, max_microseconds_delay);
          write_message(message_return);
        } else if (message.get_message_type() == protocol::GET_CONTROL_CONFIGURATION_MESSAGE_TYPE) {
          uint16_t error_constant = 0;
          uint16_t max_microseconds_delay = 0;
          auto get_control_config = [&shared_data, &error_constant, &max_microseconds_delay] () {
            error_constant = shared_data.control_configuration_data.error_constant;
            max_microseconds_delay = shared_data.control_configuration_data.max_microseconds_delay;
          };
          do_safely_sharing_data(get_control_config);
          protocol::Message message_return = protocol::Message::make_get_control_configuration_response_message(error_constant, max_microseconds_delay);
          write_message(message_return);
        } else if (message.get_message_type() == protocol::SET_CONTROL_MINMAX_CONFIGURATION_MESSAGE_TYPE) {
          int32_t minimum_steps = protocol::Message::make_int32_from_four_bytes(message.data[0], message.data[1], message.data[2], message.data[3]);
          int32_t maximum_steps = protocol::Message::make_int32_from_four_bytes(message.data[4], message.data[5], message.data[6], message.data[7]);
          auto update_control_config = [&shared_data, &minimum_steps, &maximum_steps] () {
            shared_data.control_configuration_data.minimum_steps = minimum_steps;
            shared_data.control_configuration_data.maximum_steps = maximum_steps;
          };
          do_safely_sharing_data(update_control_config);
          protocol::Message message_return = protocol::Message::make_set_control_minmax_configuration_response_message(minimum_steps, maximum_steps);
          write_message(message_return);
        } else if (message.get_message_type() == protocol::GET_CONTROL_MINMAX_CONFIGURATION_MESSAGE_TYPE) {
          int32_t minimum_steps = 0;
          int32_t maximum_steps = 0;
          auto get_control_config = [&shared_data, &minimum_steps, &maximum_steps] () {
            minimum_steps = shared_data.control_configuration_data.minimum_steps;
            maximum_steps = shared_data.control_configuration_data.maximum_steps;
          };
          do_safely_sharing_data(get_control_config);
          protocol::Message message_return = protocol::Message::make_get_control_minmax_configuration_response_message(minimum_steps, maximum_steps);
          write_message(message_return);
        }

      }
    }
  }
}
