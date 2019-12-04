#include <AS5048A.h>


AS5048A angular_3_sensor(27);
AS5048A angular_2_sensor(23);
AS5048A angular_1_sensor(29);
AS5048A linear_1_sensor(25);

int angular_1_sensor_consecutive_errors = 0;
int angular_2_sensor_consecutive_errors = 0;
int angular_3_sensor_consecutive_errors = 0;
int linear_1_sensor_consecutive_errors = 0;

#define MAX_SAMPLES  40
float samples[4][MAX_SAMPLES];
uint16_t samples_indices[4] = {0 ,0 ,0, 0};

void setup()
{
  Serial.begin(9600);
  angular_3_sensor.init();
  angular_1_sensor.init();
  linear_1_sensor.init();
  angular_2_sensor.init();
}

int test_sensor(AS5048A sensor, String id, int index, int counter){
  float value = sensor.getRotationInDegrees();
  if(!sensor.error()){
    if(samples_indices[index] < MAX_SAMPLES){
      samples[index][samples_indices[index]] = value;
      samples_indices[index]++;
    }
    //Serial.println("Got value for " + id + ": " + String(value) + ". After " + String(counter) + " errors");
    counter = 0;
  } else {
    counter++;
    sensor.getErrors();
  }
}

float compute_mean(int index){
  float* samples_in_index = samples[index];
  float mean = 0.0;
  for(int current_sample = 0; current_sample < MAX_SAMPLES; current_sample++){
    mean = ( (mean * current_sample) + samples_in_index[current_sample]) / (current_sample + 1);
  }
  return mean;
}

float compute_std(int index, float mean){
  float* samples_in_index = samples[index];
  float std = 0.0;
  for(int current_sample = 0; current_sample < MAX_SAMPLES; current_sample++){
    float squared_diff = (samples_in_index[current_sample] - mean);
    squared_diff = squared_diff * squared_diff;
    std = ( (std * current_sample) + squared_diff ) / (current_sample + 1);
  }
  return std;
}

void loop()
{
  linear_1_sensor_consecutive_errors = test_sensor(linear_1_sensor  , "Linear_1", 0, linear_1_sensor_consecutive_errors);
  angular_1_sensor_consecutive_errors = test_sensor(angular_1_sensor, "Angle_1" , 1, angular_1_sensor_consecutive_errors);
  angular_2_sensor_consecutive_errors = test_sensor(angular_2_sensor, "Angle_2" , 2, angular_2_sensor_consecutive_errors);
  angular_3_sensor_consecutive_errors = test_sensor(angular_3_sensor, "Angle_3" , 3, angular_3_sensor_consecutive_errors);
  for(int index = 0; index < 4;index++) {
      if(samples_indices[index] == MAX_SAMPLES){
        float mean = compute_mean(index);
        float std = compute_std(index, mean);
        Serial.println(String(index) + " " + String(mean) + " " + String(std));
        if(std > 10.0){
          //for(int sample = 0; sample < MAX_SAMPLES; sample++){
            //Serial.println(String(index) + " " + String(sample) + " " + String(samples[index][sample]));
          //}
          samples_indices[index] = 0;
        }
    }
  }
  delayMicroseconds(1000);
  //delayMicroseconds(1000000);
}
/*
 * Got rotation for linear1 of: 318.07
Got rotation for angular1 of: 260.76
Got rotation for angular2 of: 268.16
Got rotation for angular3 of: 217.23
____________________________

 */
