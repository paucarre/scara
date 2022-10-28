#include <ESP32Servo.h>

class ServoController {

  private:
    ESP32PWM pwm;
    Servo servo;
    int min_us = 500;
    int max_us = 2500;
    int servo_pin = 32;

  public:

    ServoController() {
    }

    void setup() {
      ESP32PWM::allocateTimer(0);
      ESP32PWM::allocateTimer(1);
      ESP32PWM::allocateTimer(2);
      ESP32PWM::allocateTimer(3);
      servo.attach(servo_pin, min_us, max_us);
      #if defined(ARDUINO_ESP32S2_DEV)
        pwm.attachPin(37, 10000);//10khz
      #else
        pwm.attachPin(27, 10000);//10khz
      #endif
      servo.setPeriodHertz(50);
    }

    void set_duty(uint32_t duty){
      	servo.write(duty);
    }

    void release() {
      	servo.detach();
      	pwm.detachPin(27);
    }

};