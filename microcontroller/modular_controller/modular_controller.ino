#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"
#include "protocol.hpp"

#include <esp_task_wdt.h>
#include <esp_int_wdt.h>


#define CENTER_MAGNETIC_SENSOR_PIN 13
#define LEFT_MAGNETIC_SENSOR_PIN 14
#define RIGH_MAGNETIC_SENSOR_PIN 12

RotaryStepper rotary_stepper(false, 27, 26);

RotaryHomer rotary_homer(CENTER_MAGNETIC_SENSOR_PIN, LEFT_MAGNETIC_SENSOR_PIN, RIGH_MAGNETIC_SENSOR_PIN);

struct ConfigurationData {
  bool dir_high_is_clockwise = rotary_stepper.get_dir_high_is_clockwise();
  uint8_t direction_pin = rotary_stepper.get_direction_pin();
  uint8_t step_pin = rotary_stepper.get_step_pin();
};

struct ActionsData {
  bool do_homing = false;
  bool do_configure = false;
};

struct SharedData {
  ActionsData  actions;
  HomingState homing_state = rotary_homer.homing_state;
  ConfigurationData configuration;
};

SharedData protocol_data;

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
  rotary_stepper.setup();
  rotary_homer.setup(rotary_stepper);
  Serial.begin(9600);// Note that Serial2 is used for communication between microcontroller and host cpu
  mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(communication, "communication", 10000, NULL, 1, NULL,  1);
  delay(1000);
  xTaskCreatePinnedToCore(control, "control", 10000, NULL, 2, NULL,  0);
  delay(1000);
}

void loop() {
}

void write_message(protocol::Message message) {
  Serial.write((const uint8_t *)message.message, (int)message.get_message_type().get_message_size());
}

void control( void * pvParameters ) {
  esp_task_wdt_feed();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  int step = 0;
  SharedData controller_data;
  for (;;) {
    auto pull_protocol_configuration = [&] () { controller_data = protocol_data; };
    do_safely_sharing_data(pull_protocol_configuration);
    if (controller_data.actions.do_homing) {
      rotary_homer.loop(rotary_stepper);
    }
    esp_task_wdt_feed();
    step++;
    if (step > 4) {
      vTaskDelay(xDelay);
      step = 0;
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
          auto activate_homing = [&] () { protocol_data.actions.do_homing = true; };
          do_safely_sharing_data(activate_homing);
          protocol::Message message_return = protocol::Message::make_homing_response_message();
          Serial.write((const uint8_t *)message_return.message, (int)message_return.get_message_type().get_message_size());
        } else if(message.get_message_type() == protocol::CONFIGURE_MESSAGE_TYPE){
          bool dir_high_is_clockwise = message.message[0 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          uint8_t direction_pin = message.message[1 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          uint8_t step_pin = message.message[2 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
          auto copy_configuration = [&, dir_high_is_clockwise, direction_pin, step_pin] () {
            protocol_data.configuration.dir_high_is_clockwise = dir_high_is_clockwise;
            protocol_data.configuration.direction_pin = direction_pin;
            protocol_data.configuration.step_pin = step_pin;
            protocol_data.actions.do_configure = true;
          };
          do_safely_sharing_data(copy_configuration);
          protocol::Message message_return = protocol::Message::make_configure_response_message();
          Serial.write((const uint8_t *)message_return.message, (int)message_return.get_message_type().get_message_size());
        }
      }
    }
    esp_task_wdt_feed();
    vTaskDelay(xDelay);
    taskYIELD();
  }
}
