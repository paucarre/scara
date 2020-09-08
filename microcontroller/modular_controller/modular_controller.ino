#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"
#include "protocol.hpp"

#include <esp_task_wdt.h>
#include <esp_int_wdt.h>


#define CENTER_MAGNETIC_SENSOR_PIN 13
#define LEFT_MAGNETIC_SENSOR_PIN 14
#define RIGH_MAGNETIC_SENSOR_PIN 12

struct ConfigurationData {
  bool dir_high_is_clockwise = false;
  uint8_t direction_pin = 27;
  uint8_t step_pin = 26;
};

struct ActionsData {
  bool do_homing = false;
  bool do_configure = false;
};

struct SharedData {
  ActionsData  actions;
  HomingState homing_state = HOMING_NOT_STARTED;
  ConfigurationData configuration;
};

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
  Serial.begin(9600); // NOTE: keep the same in host
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
 // esp_task_wdt_feed();
  SharedData controller_data;
  RotaryStepper rotary_stepper(false, 27, 26);  
  rotary_stepper.setup();
  RotaryHomer rotary_homer(CENTER_MAGNETIC_SENSOR_PIN, LEFT_MAGNETIC_SENSOR_PIN, RIGH_MAGNETIC_SENSOR_PIN);
  rotary_homer.setup(rotary_stepper);
  for (;;) {
    auto pull_protocol_configuration = [&] () { controller_data = shared_data; };
    do_safely_sharing_data(pull_protocol_configuration);
    if (controller_data.actions.do_homing) {
      rotary_homer.loop(rotary_stepper);
      auto update_homing_state = [&] () { shared_data.homing_state = rotary_homer.homing_state; };
      do_safely_sharing_data(update_homing_state);
    }
    if (controller_data.actions.do_configure) {
      rotary_stepper.configure(
        controller_data.configuration.dir_high_is_clockwise,
        controller_data.configuration.direction_pin,
        controller_data.configuration.step_pin);
      auto configuration_finished = [&] () { shared_data.actions.do_configure = false; };
      do_safely_sharing_data(configuration_finished);
    }
  }
}


protocol::ParsingResult parsing_result;

void communication( void * pvParameters ) {
  esp_task_wdt_feed();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for (;;) {
    if (Serial.available() > 0) {
      char received_byte = Serial.read();
      parsing_result = parser.parse_byte(received_byte);
      if (parsing_result.get_is_parsed()) {
        protocol::Message message = parsing_result.get_message();
        if (message.get_message_type() == protocol::HOME_MESSAGE_TYPE) {
          auto activate_homing = [&] () { shared_data.actions.do_homing = true; };
          do_safely_sharing_data(activate_homing);
          protocol::Message message_return = protocol::Message::make_homing_response_message();
          write_message(message_return);
        } else if(message.get_message_type() == protocol::CONFIGURE_MESSAGE_TYPE){
          bool dir_high_is_clockwise = message.message[0 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          uint8_t direction_pin = message.message[1 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          uint8_t step_pin = message.message[2 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          auto copy_configuration = [&, dir_high_is_clockwise, direction_pin, step_pin] () {
            shared_data.configuration.dir_high_is_clockwise = dir_high_is_clockwise;
            shared_data.configuration.direction_pin = direction_pin;
            shared_data.configuration.step_pin = step_pin;
            shared_data.actions.do_configure = true;
          };
          do_safely_sharing_data(copy_configuration);
          protocol::Message message_return = protocol::Message::make_configure_response_message();
          write_message(message_return);
        } else if(message.get_message_type() == protocol::HOMING_STATE_MESSAGE_TYPE){                   
          //HomingState homing_state = HomingState::HOMING_NOT_STARTED;
          //auto get_homing_state = [&homing_state, shared_data] () { homing_state = shared_data.homing_state; };
          //do_safely_sharing_data(get_homing_state);
          protocol::Message message_return = protocol::Message::make_homing_state_response_message(0x12);
          write_message(message_return);
        }
      }
    }
    esp_task_wdt_feed();
    vTaskDelay(xDelay);
    taskYIELD();
  }
}
