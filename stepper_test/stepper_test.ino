#include "driver/gpio.h"

#define RXD2 16
#define TXD2 17

const uint32_t PLUSE_PER_REVOLUTION = 1000;
const uint32_t MAX_COUNTS = PLUSE_PER_REVOLUTION / 2;
const int GPIO2 = 2;
const int GPIO15 = 15;

uint32_t counter = MAX_COUNTS;
bool direction = LOW;
uint32_t delay_microseconds = 2;

void setup() {
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode (GPIO2, OUTPUT);
  pinMode (GPIO15, OUTPUT);
  digitalWrite (GPIO15, direction);
}

void loop() {
  //Serial1.println("1Control Loop - control call");
  //Serial2.println("2Control Loop - control call");
  delay_microseconds = 2000;// + ( 100 * (MAX_COUNTS - counter) );
  digitalWrite(GPIO2, HIGH);
  Serial2.println("2 - Control Loop - control call");
  //delayMicroseconds(delay_microseconds);
  digitalWrite(GPIO2, LOW);
  delayMicroseconds(delay_microseconds);
  //counter-- ;
  //if(counter == 0) {
  //  counter = MAX_COUNTS;
  //  digitalWrite(GPIO15, direction);
  //  direction = !direction;
  //}
}