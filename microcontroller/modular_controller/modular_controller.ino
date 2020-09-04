#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"
#include "protocol.hpp"

#include <esp_task_wdt.h>
#include <esp_int_wdt.h>


#define CENTER_MAGNETIC_SENSOR_PIN 13
#define LEFT_MAGNETIC_SENSOR_PIN 14
#define RIGH_MAGNETIC_SENSOR_PIN 12
// axis 3
// RotaryStepper rotary_stepper(false, 27, 26);
// axis 2
// RotaryStepper rotary_stepper(false, 27, 26);
// axis 1
RotaryStepper rotary_stepper(false, 27, 26);

RotaryHomer rotary_homer(CENTER_MAGNETIC_SENSOR_PIN, LEFT_MAGNETIC_SENSOR_PIN, RIGH_MAGNETIC_SENSOR_PIN);

SemaphoreHandle_t mutex;
volatile bool do_homing = false;

protocol::Parser parser;

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

void control( void * pvParameters ){
  esp_task_wdt_feed();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  int step = 0;
  bool local_do_homing = false;
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {
      local_do_homing = do_homing;
      xSemaphoreGive(mutex);
    }
    if(local_do_homing){
      rotary_homer.loop(rotary_stepper);
    }
    esp_task_wdt_feed();
    step++;
    if(step > 4){
      vTaskDelay(xDelay);
      step = 0;
    }
    //taskYIELD();
  }
}


protocol::ParsingResult parsing_result;
//parsing_result = parser.parse_byte(message[0]);

void communication( void * pvParameters ){
  esp_task_wdt_feed();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for(;;){
    char received_byte = Serial.read();
    parsing_result = parser.parse_byte(received_byte);
    if(parsing_result.get_is_parsed()){
      protocol::Message message = parsing_result.get_message();
      if(message.get_message_type() == protocol::HOME_MESSAGE_TYPE){
        if(xSemaphoreTake(mutex, 10) == pdTRUE) {
          if(!do_homing) {
            do_homing = true;
          }
          xSemaphoreGive(mutex);
        }
      } else if(message.get_message_type() == protocol::CONFIGURE_MESSAGE_TYPE){
        bool dir_high_is_clockwise = message.message[0 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
        uint8_t direction_pin = message.message[1 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
        uint8_t step_pin = message.message[2 + protocol::MESSAGE_DATA_OFFSET_IN_BYTES];
        if(xSemaphoreTake(mutex, 10) == pdTRUE) {
          rotary_stepper.configure(dir_high_is_clockwise, direction_pin, step_pin);         
          xSemaphoreGive(mutex);
        }
      }
      if(message.get_message_type() != protocol::UNDEFINED_MESSAGE_TYPE){        
        protocol::Message message_ack = protocol::Message::make_response_message(message);
        Serial.write(message_ack.message);
      }
    }
    delayMicroseconds(100);
    esp_task_wdt_feed();
    vTaskDelay(xDelay);
    taskYIELD();
  }
}
