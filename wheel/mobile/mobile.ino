

#include <Stepper.h>

const int steps_per_revolution = 4096;  
Stepper stepper_4(steps_per_revolution, 53, 49, 51, 47);
Stepper stepper_3(steps_per_revolution, 37, 33, 35, 31);
Stepper stepper_2(steps_per_revolution, 45, 41, 43,  39);
Stepper stepper_1(steps_per_revolution, 29, 25, 27, 23);
int stepper_1_forward = 1;
int stepper_2_forward = -1;
int stepper_3_forward = -1;
int stepper_4_forward = -1;

void setup() {
  stepper_1.setSpeed(6);
  stepper_2.setSpeed(6);
  stepper_3.setSpeed(6);
  stepper_4.setSpeed(6);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()){
    uint8_t actuator = Serial.read();
    uint8_t steps = 100;//Serial.readString().toInt();
    for(int i=0;i<steps;i++){
      if(actuator == 'L'){
        left();    
      } else if(actuator == 'R'){
        right();
      } else if(actuator == 'C'){
        clockwise();
      } else if(actuator == 'A'){
        anticlockwise();
      } else if(actuator == 'F'){
        forward();
      } else if(actuator == 'B'){
        backward();
      }
    }
  }
}

void left() {
  stepper_1.step(-stepper_1_forward);
  stepper_2.step(stepper_2_forward);
  stepper_3.step(-stepper_3_forward);
  stepper_4.step(stepper_4_forward);
}
void right() {
    stepper_1.step(stepper_1_forward);
    stepper_2.step(-stepper_2_forward);
    stepper_3.step(stepper_3_forward);
    stepper_4.step(-stepper_4_forward);
}
void clockwise() {
    stepper_1.step(stepper_1_forward);
    stepper_2.step(-stepper_2_forward);
    stepper_3.step(-stepper_3_forward);
    stepper_4.step(stepper_4_forward);
}
void anticlockwise() {
    stepper_1.step(-stepper_1_forward);
    stepper_2.step(stepper_2_forward);
    stepper_3.step(stepper_3_forward);
    stepper_4.step(-stepper_4_forward);
}
void forward() {
    stepper_1.step(stepper_1_forward);
    stepper_2.step(stepper_2_forward);
    stepper_3.step(stepper_3_forward);
    stepper_4.step(stepper_4_forward);
}
void backward() {
    stepper_1.step(-stepper_1_forward);
    stepper_2.step(-stepper_2_forward);
    stepper_3.step(-stepper_3_forward);
    stepper_4.step(-stepper_4_forward);
}
