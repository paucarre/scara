#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"

#define CENTER_MAGNETIC_SENSOR_PIN 14
#define LEFT_MAGNETIC_SENSOR_PIN 12
#define RIGH_MAGNETIC_SENSOR_PIN 13

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
  xTaskCreatePinnedToCore(control, "control", 10000, NULL, 1, NULL,  0); 
  delay(1000); 
}

void loop() {
}

void control( void * pvParameters ){
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {
      if(do_homing){
        rotary_homer.loop(rotary_stepper);
        delay(1);
      }
      xSemaphoreGive(mutex);
    }
  }
}

void communication( void * pvParameters ){
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {      
      if(!do_homing) {        
        do_homing = true;
      }
      xSemaphoreGive(mutex);
    }
  }
}
