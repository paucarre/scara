#include <ESP32Servo.h>

class ServoController {

  private:
    ESP32PWM pwm;
    Servo servo;
    int min_us = 1000;
    int max_us = 2000;
    int servo_pin = 32;

  public:

    ServoController() {
    }

    void setup() {
      // Allow allocation of all timers
      ESP32PWM::allocateTimer(0);
      ESP32PWM::allocateTimer(1);
      ESP32PWM::allocateTimer(2);
      ESP32PWM::allocateTimer(3);
      #if defined(ARDUINO_ESP32S2_DEV)
        pwm.attachPin(37, 10000);//10khz
      #else
        pwm.attachPin(27, 10000);//10khz
      #endif
      Serial.begin(115200);
      servo.setPeriodHertz(50);
    }

    void set_angle(uint16_t angle){
      	servo.write(angle);
    }

    void release() {
      	servo.detach();
      	pwm.detachPin(27);
    }

};