#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1015.h>
#include "rotary_stepper.hpp"

const uint16_t SAMPLES = 1;
const uint16_t STEPPER_STEPS = 25000;

const uint16_t STEPPER_SAMPLER_STEPS = 4;

Adafruit_ADS1115 ads;

int16_t sensor_0_diff[STEPPER_STEPS];
int16_t sensor_1_diff[STEPPER_STEPS];

uint16_t iteration = 0;
bool max_min_finished = false;
bool diff_finished = false;

uint16_t sensor_0_max = 0;
uint16_t sensor_0_min = 0xFFFFF;
uint16_t sensor_1_max = 0;
uint16_t sensor_1_min = 0xFFFFF;

float zero_angle = -1.0f;
RotaryStepper rotary_stepper(false, 26, 27);

void setup(void) 
{
  Serial.begin(9600);
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();  
  rotary_stepper.setup();
  rotary_stepper.apply_direction(true);
  for(uint16_t i = 0; i < STEPPER_STEPS;i++) {
    sensor_0_diff[i] = 0;
    sensor_1_diff[i] = 0;
  }
  for(uint8_t i = 0; i < 30;i++) { 
    uint16_t adc_0 = ads.readADC_SingleEnded(0);
    uint16_t adc_1 = ads.readADC_SingleEnded(1);
    rotary_stepper.step();
  }
      
}

/*
x is between 0 and 5 
x = -(2 * x) + 5.0

r2/r1=2 ;r2= 2*r1
(1+r2/r1)*v=5 ; (1+2)v = 5 ; v = 5/3

(5*z2)/(z1+z2)=5/3 ; z2/(z1+z2)=1/3
3z2=z1+z2;z1=2z2
*/

double transform_to_waveform(uint16_t signal_value, uint16_t  signal_max, uint16_t  signal_min){
    double max_min_distance = ( signal_max - signal_min ) / 2;
    double singal_a_transformed = (signal_value - max_min_distance - signal_min) / max_min_distance;
    return singal_a_transformed;
}
    
void loop(void) 
{
  rotary_stepper.step();
  
  uint32_t sensor_0 = 0;
  uint32_t sensor_1 = 0;
  for(uint8_t i = 0; i < SAMPLES;i++){
    sensor_0 += ads.readADC_SingleEnded(0);
    sensor_1 += ads.readADC_SingleEnded(1);
  }
  sensor_0 /= SAMPLES;
  sensor_1 /= SAMPLES;
  if(sensor_0 > sensor_0_max){
    sensor_0_max = sensor_0;
  }
  if(sensor_0 < sensor_0_min){
    sensor_0_min = sensor_0;
  }
  if(sensor_1 > sensor_1_max){
    sensor_1_max = sensor_1;
  }
  if(sensor_1 < sensor_1_min){
    sensor_1_min = sensor_1;
  }

 

  if(iteration == 0 && max_min_finished){
    double signal_0_wave = transform_to_waveform(sensor_0, sensor_0_max, sensor_0_min);
    double signal_1_wave = transform_to_waveform(sensor_1, sensor_1_max, sensor_1_min);
    float angle = atan2(signal_0_wave, signal_1_wave) + (M_PI);
    zero_angle = ( (int) ( (angle * STEPPER_STEPS) / (2. * M_PI) )) % STEPPER_STEPS;
    //zero_angle = angle;
  }
  if(iteration % 100 == 0){    
    if(!max_min_finished) {
     Serial.print("sensor_0");
     Serial.print(",");
     Serial.print("sensor_0_min");
     Serial.print(",");
     Serial.print("sensor_0_max");
     Serial.print(",");
     Serial.print("sensor_1");
     Serial.print(",");
     Serial.print("sensor_1_min");
     Serial.print(",");
     Serial.print("sensor_1_max");
     Serial.println("");
     Serial.print(sensor_0);
     Serial.print(",");
     Serial.print(sensor_0_min);
     Serial.print(",");
     Serial.print(sensor_0_max);
     Serial.print(",");
     Serial.print(sensor_1);
     Serial.print(",");
     Serial.print(sensor_1_min);
     Serial.print(",");
     Serial.print(sensor_1_max);
     Serial.println("");
    } else {          
     double signal_0_wave = transform_to_waveform(sensor_0, sensor_0_max, sensor_0_min);
     double signal_1_wave = transform_to_waveform(sensor_1, sensor_1_max, sensor_1_min);
     double angle = atan2(signal_0_wave, signal_1_wave) + (M_PI);
     //angle = angle - zero_angle;
     //if(angle < 0.0) {
     // angle +=  2. * M_PI;
     ///}
     
     double singal_0_computed = sin(angle);
     double singal_1_computed = cos(angle);
     

     double stepper_angle = (double)( ( (STEPPER_STEPS - iteration - (int) zero_angle) % STEPPER_STEPS ) * 2. * M_PI) / (double)STEPPER_STEPS;
     double singal_0_stepper = sin(stepper_angle);
     double singal_1_stepper = cos(stepper_angle);
     if(!diff_finished){
        sensor_0_diff[iteration] = (int16_t) ( (singal_0_stepper - signal_0_wave) * 10000.0);
        sensor_1_diff[iteration] = (int16_t) ( (singal_1_stepper - signal_1_wave) * 10000.0);
     } else {
       //signal_0_wave += (((double)sensor_0_diff[iteration]) / 10000.0);
       //signal_1_wave += (((double)sensor_1_diff[iteration]) / 10000.0);
       //double angle = atan2(signal_0_wave, signal_1_wave) + (M_PI);
       //angle = angle - zero_angle;
       //if(angle < 0.0) {
       // angle +=  2. * M_PI;
       //} 
     }
      /*
     double error = abs(angle - stepper_angle);
     if(error >= (2. * M_PI) - 0.3){
      error -= 2. * M_PI;
     }
     Serial.println(error * 10);
     */
    
     Serial.print("angle");
     Serial.print(",");
     Serial.print("stepper_angle");
     Serial.print(",");
     Serial.print("zero_angle");
     Serial.println("");
     Serial.print(angle);
     Serial.print(",");
     Serial.print(stepper_angle);
     Serial.print(",");
     Serial.print(zero_angle);
     Serial.println("");
     
      
     /*
     Serial.print("signal_0_wave");
     Serial.print(",");
     Serial.print("singal_0_computed");
     Serial.print(",");
     Serial.print("singal_0_stepper");
     Serial.print(",");
     Serial.print("signal_1_wave");
     Serial.print(",");
     Serial.print("singal_1_computed");
     Serial.print(",");
     Serial.print("singal_1_stepper");
     Serial.println("");
     Serial.print(signal_0_wave * 5);
     Serial.print(",");
     Serial.print(singal_0_computed * 5);
     Serial.print(",");
     Serial.print(singal_0_stepper * 5);
     Serial.print(",");
     Serial.print(signal_1_wave * 5);
     Serial.print(",");
     Serial.print(singal_1_computed * 5);
     Serial.print(",");
     Serial.print(singal_1_stepper * 5);
     Serial.println("");
      */
    }
  }
  
  iteration = (iteration + 1 ) % STEPPER_STEPS;
  if(iteration == 0){
    if(max_min_finished){
      diff_finished = true;
    } else {
      max_min_finished = true;
    }
  }
}
/*
#include "rotary_stepper.hpp"
#include "rotary_homer.hpp"

#define READ_INDUCTANCE_ZERO_SENSOR 14

RotaryStepper rotary_stepper(false, 27, 26);
RotaryHomer rotary_homer(READ_INDUCTANCE_ZERO_SENSOR);

SemaphoreHandle_t mutex;
volatile bool do_homing = false;

void setup() {
  rotary_stepper.setup();
  rotary_homer.setup(rotary_stepper);
  Serial.begin(9600);
  mutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(communication, "communication", 10000, NULL, 1, NULL,  1); 
  delay(500); 
  xTaskCreatePinnedToCore(control, "control", 10000, NULL, 1, NULL,  0); 
  delay(500); 
}

void loop() {
}

void control( void * pvParameters ){
  for(;;){
    if(xSemaphoreTake(mutex, 10) == pdTRUE) {
      if(do_homing){
        rotary_homer.loop(rotary_stepper);
        Serial.println(String(rotary_homer.homing_state));
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
        delay(20000);
        do_homing = true;
      }
      xSemaphoreGive(mutex);
    }
  }
}*/
