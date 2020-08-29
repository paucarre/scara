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

void setup() {
  rotary_stepper.setup();
  rotary_homer.setup(rotary_stepper);
  Serial.begin(9600);
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
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {
      if(do_homing){        
        rotary_homer.loop(rotary_stepper);
      }
      xSemaphoreGive(mutex);
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

/*
void message_generation_test(protocol::MessageType message_type) {
    uint8_t data[0];
    uint8_t message[message_type.get_message_size()];
    protocol::MessageFactory::get_message_data(message_type, data, message);
    assert(message[0] == protocol::Parser::START_FLAG);
    assert(message[1] == message_type.get_label());
    assert(message[2] == 0x00 ^ message_type.get_label());
    assert(message[3] == protocol::Parser::END_FLAG);
    assert(sizeof(message)/sizeof(*message) == 4);
    std::cout << "SUCCESS -- MESSAGE GENERATION" << std::endl;
}
*/
void communication( void * pvParameters ){
  esp_task_wdt_feed();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {      
      if(!do_homing) {        
        do_homing = true;
      }
      xSemaphoreGive(mutex);
    }
    delayMicroseconds(1000);
    esp_task_wdt_feed();
    vTaskDelay(xDelay);
    taskYIELD();
  }
}
