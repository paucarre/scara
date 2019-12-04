#include <AS5048A.h>
#include "math.h"


typedef struct  {
  uint16_t linear_1;
  uint16_t angular_1;
  uint16_t angular_2;
  uint16_t angular_3;
} communication_state;

typedef struct  {
  float linear_1;
  float angular_1;
  float angular_2;
  float angular_3;
} continious_state;

class StateManager {

  public:
  
    static float to_continious(uint16_t communication) { 
       float continious = (float) communication;
       return continious / 8;
    }
    
    static uint16_t to_communication(float continious) {
      unsigned int communication_int = floor(8 * continious);
      return (uint16_t) communication_int;
    }
    
    static continious_state to_continious(communication_state communication) {
      continious_state continious;
      continious.linear_1   = StateManager::to_continious(communication.linear_1);
      continious.angular_1  = StateManager::to_continious(communication.angular_1);
      continious.angular_2  = StateManager::to_continious(communication.angular_2);
      continious.angular_3  = StateManager::to_continious(communication.angular_3);
      return continious;
    }

    static communication_state to_communication(continious_state continious) {
      communication_state communication;
      communication.linear_1   = StateManager::to_communication(continious.linear_1);
      communication.angular_1  = StateManager::to_communication(continious.angular_1);
      communication.angular_2  = StateManager::to_communication(continious.angular_2);
      communication.angular_3  = StateManager::to_communication(continious.angular_3);
      return communication;
    }
  
};

class Communication {
  /*
    Packet format (11 bytes):
        Command (2 bytes)
            0xCFFF: write target state
            0xEFFF: read current state
            0xEFFF: response to read current state
            0xDFFF: read target state
            0xDFFF: response to read target state
        State (8 bytes)
            linear_1 (2 bytes)
            angular_1 (2 bytes)
            angular_2 (2 bytes)
            angular_3 (2 bytes)
        Error detection code (1 byte)
   */
  protected:
    static const uint16_t WRITE_TARGET_STATE_HEADER = 0xCFFF;
    static const uint16_t READ_CURRENT_STATE_HEADER = 0xEFFF;
    static const uint16_t READ_TARGET_STATE_HEADER  = 0xDFFF;
    static const uint16_t UNDEFINED_HEADER  = 0x0000;
    
    static const uint8_t HEADER_STATE_NO_BYTE_RECEIVED   = 0x00;
    static const uint8_t HEADER_STATE_HIGH_BYTE_RECEIVED = 0x01;
    static const uint8_t HEADER_STATE_LOW_BYTE_RECEIVED  = 0x02;

    uint8_t compute_check_byte(communication_state s);
    uint16_t read_uint16_t();
    void write_target_state(continious_state& target_state);
    void process_command_type_state(uint8_t byte_read);
    
    void read_state(communication_state s, const uint16_t header);
    void send_uint16_t(uint16_t data);
    void try_to_read_command_type();
    
  public:
    Communication() ;
    void communication_loop(continious_state current_state, continious_state& target_state);
    uint16_t command_type = UNDEFINED_HEADER;
    uint8_t command_parsing_state = HEADER_STATE_NO_BYTE_RECEIVED;
  
};

Communication::Communication() {
}

uint8_t Communication::compute_check_byte(communication_state s) {
  return (s.linear_1 ^ s.angular_1 ^ s.angular_2 ^ s.angular_3) & 0x00FF;
}

uint16_t Communication::read_uint16_t() {
  uint8_t data_high = Serial2.read();
  uint8_t data_low = Serial2.read();
  return data_high << 8 | data_low;
}
    
void Communication::write_target_state(continious_state& target_state) {
  communication_state received_target_state;
  bool success = true;
  if(success) {
    while(Serial2.available() < 9);
    received_target_state.linear_1  = read_uint16_t();
    received_target_state.angular_1 = read_uint16_t();
    received_target_state.angular_2 = read_uint16_t();
    received_target_state.angular_3 = read_uint16_t();
    uint8_t computed_check = compute_check_byte(received_target_state);
    uint8_t read_check = Serial2.read();
    if(computed_check == read_check) {
      target_state = StateManager::to_continious(received_target_state);
    } else {
      Serial.println("ERROR!!! : Wrong error code in serial packet. Received " + String(read_check, HEX) + " | Computed " + String(computed_check, HEX));
    }
  } else {
    Serial.println("ERROR!!! : Could not find packet header.");
  }
}

void Communication::process_command_type_state(uint8_t byte_read) {
  switch(command_parsing_state) {
    case Communication::HEADER_STATE_NO_BYTE_RECEIVED:
      command_type = byte_read << 8;
      command_parsing_state = Communication::HEADER_STATE_HIGH_BYTE_RECEIVED; 
      break;
    case Communication::HEADER_STATE_HIGH_BYTE_RECEIVED:
      command_type |= byte_read;
      command_parsing_state = Communication::HEADER_STATE_LOW_BYTE_RECEIVED;
      break;
  }
}

void Communication::try_to_read_command_type() {
  if(Serial2.available()){
    //Serial.println("Something available in serial 3");
    uint8_t command_type_byte = Serial2.read();
    //Serial.println("command_type_byte: " + String(command_type_byte));
    if(command_type_byte == 0xFF || command_type_byte == 0xEF || command_type_byte == 0xDF || command_type_byte == 0xCF) {
      process_command_type_state(command_type_byte);
    } else {
      // error, resetting parsing
      command_type = 0x0000;
      command_parsing_state = Communication::HEADER_STATE_NO_BYTE_RECEIVED;
    }
  }
}

void Communication::send_uint16_t(uint16_t data){
  uint8_t data_high = (data >> 8) & 0x00FF;
  Serial2.write(data_high);
  //Serial.print(data_high, HEX);
  //Serial.print(" ");
  uint8_t data_low = data & 0x00FF;
  Serial2.write(data_low);
  //Serial.print(data_low, HEX);
  //Serial.print(" ");
}

void Communication::read_state(communication_state s, const uint16_t header){
  send_uint16_t(header);
  send_uint16_t(s.linear_1);
  send_uint16_t(s.angular_1);
  send_uint16_t(s.angular_2);
  send_uint16_t(s.angular_3);
  uint8_t check = compute_check_byte(s);
  //Serial.print(check, HEX);
  Serial2.write(check);
  //Serial.println("");
}

void Communication::communication_loop(continious_state current_state, continious_state& target_state)
{
  //Serial.println("communication_loop");
  try_to_read_command_type();
  //if(command_type != 0){
    //Serial.println("command_type: " + String(command_type));
  //}
  switch(command_type){
    case Communication::READ_CURRENT_STATE_HEADER:
      read_state(StateManager::to_communication(current_state), Communication::READ_CURRENT_STATE_HEADER);
      break;
    case Communication::READ_TARGET_STATE_HEADER:
      read_state(StateManager::to_communication(target_state), Communication::READ_TARGET_STATE_HEADER);
      break;
    case Communication::WRITE_TARGET_STATE_HEADER:
      write_target_state(target_state);
      break;
  }
  if(command_parsing_state == HEADER_STATE_LOW_BYTE_RECEIVED){
    command_type = 0x0000;
    command_parsing_state = HEADER_STATE_NO_BYTE_RECEIVED;
  }
  
}

uint16_t sck_pin  = 13;
uint16_t miso_pin = 12;
uint16_t mosi_pin = 11;

#define A1 14

#define BOTTOM_INPUT_END_STOP A0
#define TOP_INPUT_END_STOP A1


class StandardStepper {
  private:
  public:
    uint16_t direction_pin;
    uint16_t step_pin;
    bool dir_high_is_clockwise;
    StandardStepper(bool dir_high_is_clockwise_, int direction_pin_, int step_pin_): 
      dir_high_is_clockwise(dir_high_is_clockwise_), direction_pin(direction_pin_), step_pin(step_pin_) {
    }
    void setup();
    void step();
    void apply_direction(bool rotate_clockwise);
    
};

void StandardStepper::setup() {
  pinMode(this->direction_pin, OUTPUT);
  pinMode(this->step_pin, OUTPUT);
  digitalWrite(this->step_pin, HIGH);
  this->apply_direction(true);
}

void StandardStepper::apply_direction(bool rotate_clockwise) {
  //Serial.println( "rotate_clockwise:" + String(rotate_clockwise) + " --- is_clockwise : " + String(this->dir_high_is_clockwise) );
  if( (this->dir_high_is_clockwise && rotate_clockwise) || ((!rotate_clockwise) && (!this->dir_high_is_clockwise)) ) {
    //Serial.println("Setting direction to High");
    digitalWrite(this->direction_pin, HIGH);
  } else {
    //Serial.println("Setting direction to LOW");
    digitalWrite(this->direction_pin, LOW);
  }
}

void StandardStepper::step() {
  digitalWrite(this->step_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(this->step_pin, LOW);
  delayMicroseconds(5);
}


#define LIMIT_SWITCH_ENABLED       1
#define LIMIT_SWITCH_DISABLED      0
class LimitSwitch {
    int limit_switch_state;
  public:
    LimitSwitch(int limit_switch_state_): limit_switch_state(limit_switch_state_) {

    }
    LimitSwitch(): limit_switch_state(LIMIT_SWITCH_DISABLED) {

    }

    bool became_turned_on(int analog_sensor_read);
};

bool LimitSwitch::became_turned_on(int analog_sensor_read) {
  if (analog_sensor_read > 50 && this->limit_switch_state == LIMIT_SWITCH_ENABLED) {
    this->limit_switch_state = LIMIT_SWITCH_DISABLED;
  } else if (analog_sensor_read < 50 && this->limit_switch_state == LIMIT_SWITCH_DISABLED) {
    this->limit_switch_state = LIMIT_SWITCH_ENABLED;
    return true;
  }
  return false;
}
  
#define ONE_SECOND_IN_MICROSECONDS  1000000
#define ONE_MILLISECONDSECOND_IN_MICROSECONDS  1000

class LinearController {
  public:
    float mm_per_degree = 270.0 / 678.0;
    bool is_initialized;
    float current_loop;
    float previous_angle;
    float bottom_angle;
    float top_angle;
    float max_height;
    bool has_to_cross_180_to_increase_loop;
    bool has_to_cross_180_to_reduce_loop;
    uint32_t loops;
    AS5048A* angular_sensor;
    StandardStepper* stepper;
    float height_value;
    
    LinearController() {
      is_initialized = false;
    }
    
    LinearController(AS5048A* angular_sensor_, StandardStepper* stepper_, float bottom_angle_, float top_angle_,
      uint32_t loops_): 
      angular_sensor(angular_sensor_), stepper(stepper_), bottom_angle(bottom_angle_), top_angle(top_angle_),
      loops(loops_) {
        current_loop = 0;
        previous_angle = bottom_angle;
        is_initialized = true;
        has_to_cross_180_to_increase_loop = previous_angle <= 170;
        has_to_cross_180_to_reduce_loop = previous_angle >= 190;
        max_height = distance_in_mm(top_angle, loops);
    }
    
    LinearController& operator=(const LinearController& other)
    {
        angular_sensor = other.angular_sensor;
        bottom_angle = other.bottom_angle;
        top_angle = other.top_angle;
        loops = other.loops;
        stepper = other.stepper;
        
        current_loop = other.current_loop;
        previous_angle = other.previous_angle;
        is_initialized = true;
        has_to_cross_180_to_increase_loop = previous_angle <= 170;
        has_to_cross_180_to_reduce_loop = previous_angle >= 190;
        max_height = distance_in_mm(top_angle, loops);
    }
    
    float distance_in_mm(float angle, uint32_t loop_count);
    bool control(float target);
    void update_sensor();
    float distance_in_mm();
};

void LinearController::update_sensor() {
  float current_angle = angular_sensor->getRotationInDegrees();

  // increase loop
  if(has_to_cross_180_to_increase_loop && current_angle > 180){    
    has_to_cross_180_to_increase_loop = false;
  }
  if((!has_to_cross_180_to_increase_loop) && 
    current_angle < 10 &&
    previous_angle > 350){
      current_loop ++;
      has_to_cross_180_to_increase_loop = true;
  }
  
  // reduce loop
  if(has_to_cross_180_to_reduce_loop && current_angle < 180){
    has_to_cross_180_to_reduce_loop = false;
  }
  if((!has_to_cross_180_to_reduce_loop) && 
    current_angle > 350 &&
    previous_angle < 10){
      current_loop --;
      has_to_cross_180_to_reduce_loop = true;
  }
  
  height_value = this->distance_in_mm();
  previous_angle = current_angle;
}

float LinearController::distance_in_mm() {
  return distance_in_mm(previous_angle, current_loop);
}

float LinearController::distance_in_mm(float angle, uint32_t loop_count) {
  float sum_of_degrees = (loop_count * 360) + angle - bottom_angle;
  if(sum_of_degrees < 0) {
    sum_of_degrees = 0; 
  }
  return sum_of_degrees * mm_per_degree;
}

bool LinearController::control(float target) {
  this->update_sensor();
  bool in_target = abs(target - height_value) < 0.5;
  if(!in_target) {
    bool down_direction = target < height_value;
    if( (height_value > 5.0 || !down_direction) && (height_value < max_height - 5.0 || down_direction) ) {
      stepper->apply_direction(down_direction);
      stepper->step();
    }
  }
  return in_target;
}

typedef struct  {
  float value;
  bool is_error;
  uint16_t error_value;
} sensor_read_result;

#define UNDEFINED_DISTANCE 1000.0

class AngularController {
  protected:
    sensor_read_result try_to_read_rotation_in_degrees(uint8_t attempts);
  public:
    int id;
    float offset;
    StandardStepper* standard_stepper;
    AS5048A* angular_sensor;
    bool angular_sensor_is_anticlockwise;
    float angle_value;
    AngularController(int id_, AS5048A* angular_sensor_, StandardStepper* standard_stepper_, float offset_, bool angular_sensor_is_anticlockwise_): 
      id(id_), angular_sensor(angular_sensor_), standard_stepper(standard_stepper_), offset(offset_), angular_sensor_is_anticlockwise(angular_sensor_is_anticlockwise_) {
    }
    sensor_read_result control_signal(float target_angle);
    float difference(float left_angle, float right_angle);
    sensor_read_result angle();
    void home_at_position();
    uint32_t control(float target);
};


void AngularController::home_at_position() {
  float samples = 0.0;
  offset = 0.0;
  do{
    sensor_read_result sensor_read = try_to_read_rotation_in_degrees(1000);
    if(!sensor_read.is_error){
      offset = ( (offset * samples) + sensor_read.value ) / (samples + 1.0);
      samples++;
    }
  } while (samples < 20);
}

sensor_read_result AngularController::try_to_read_rotation_in_degrees(uint8_t max_attempts) {
  sensor_read_result result;
  do {
    result.value = angular_sensor->getRotationInDegrees();
    result.is_error = angular_sensor->error();
    if(result.is_error){
      result.error_value = angular_sensor->getErrors();
    }
    //Serial.println(String(id) + "  " + String(result.is_error) + "  " + String(max_attempts));
    max_attempts--;
  } while(max_attempts > 0 && result.is_error);
  return result;
}

sensor_read_result AngularController::angle() {
  sensor_read_result angle_read_result = this->try_to_read_rotation_in_degrees(1000);
  if(!angle_read_result.is_error) {
    if(angular_sensor_is_anticlockwise) {
      angle_read_result.value = angle_read_result.value - this->offset;
    } else {
      angle_read_result.value = 360 - (angle_read_result.value - this->offset);
    }
    while(angle_read_result.value > 360) {
      angle_read_result.value -= 360;
    }
    while(angle_read_result.value < 0) {
      angle_read_result.value += 360;
    }
    this->angle_value = angle_read_result.value;
    //Serial.println(String(id) + "  " + String(this->angle_value));
  } 
  return angle_read_result;
}
/*
 * Angular difference fullfills the following requirements:
 *    5 - 350 =  15
 *  350 -   5 = -15
 *   60 -  80 = -20
 *   80 -  60 =  20
 */
float AngularController::difference(float leftAngle, float rightAngle)
{
    float difference = leftAngle - rightAngle;
    if(difference > 180.0){
        difference -= 360;
    } else if(difference < - 180.0) {
        difference += 360;
    }
    return difference;
}

sensor_read_result AngularController::control_signal(float target_angle) {
 sensor_read_result current_angle = this->angle();
 if(!current_angle.is_error) {
    current_angle.value = this->difference(target_angle, current_angle.value);
    //Serial.println(String(this->id) + " | Current: " + String(current_angle.value) + " | " + "Target: " + String(target_angle));
 } else {
   Serial.println(String(this->id) + " Error reading sensor.");
 }
 return current_angle;
}


uint32_t AngularController::control(float target) {
  sensor_read_result distance = control_signal(target);
  uint32_t delay_in_loop = 0;
  if(!distance.is_error){
    bool in_target_angular = abs(distance.value) <= 1.0;
    if(!in_target_angular) {
      if(abs(distance.value) < 10) {
        delay_in_loop += 20 * (10 - abs(distance.value));
      }
      //Serial.println("Direction value " + String(distance.value));
      standard_stepper->apply_direction(distance.value > 0);
      standard_stepper->step();
    }
  }
  return delay_in_loop;
}


LimitSwitch limit_switch_bottom(LIMIT_SWITCH_ENABLED);
LimitSwitch limit_switch_top;
StandardStepper angular_1_stepper(
          true, // clockwise is dir high
           4, // direction pin
           5 // step pin
    );
StandardStepper angular_2_stepper(
          true, // clockwise is dir high
           6, // direction pin
           7 // step pin
    );
StandardStepper angular_3_stepper(
          false, // clockwise is dir high
           8, // direction pin
           9 // step pin
    );

StandardStepper linear_1_stepper(
          true, // clockwise is dir high
           2, // direction pin
           3 // step pin
    );

Communication communication;

AS5048A angular_3_sensor(27);
AS5048A angular_2_sensor(23);
AS5048A angular_1_sensor(29);
AS5048A linear_1_sensor(25);
   
    
AngularController angular_1_controller(1, &angular_1_sensor, &angular_1_stepper, 329.04 + 90.0 - 360.0, false);
AngularController angular_2_controller(2, &angular_2_sensor, &angular_2_stepper, 319.31, true );
AngularController angular_3_controller(3, &angular_3_sensor, &angular_3_stepper, 321.23, true);

LinearController home_linear() {
  delay(500);
  float top_rotation_in_degrees;
  uint8_t loops = 0;
  Serial.println("Placing base so that it contacts bottom limit shwitch.");
  int limit_switch_input_bottom = 100;
  linear_1_stepper.apply_direction(true);
  while(limit_switch_input_bottom > 10.0) {
    limit_switch_input_bottom = analogRead(BOTTOM_INPUT_END_STOP);
    delayMicroseconds(100);
    linear_1_stepper.step();
  }
  linear_1_stepper.apply_direction(false);
  Serial.println("Bottom limit switch detected.");
  float bottom_rotation_in_degrees = linear_1_sensor.getRotationInDegrees();
  Serial.println(String("Bottom rotation in degrees: ") + String(bottom_rotation_in_degrees));
  LinearController homing_controller(&linear_1_sensor, &linear_1_stepper, bottom_rotation_in_degrees, 0.0, 0.0);
  Serial.println("Moving platform up until top limit switch is detected.");
  top_rotation_in_degrees = bottom_rotation_in_degrees;
  float previous_rotation_in_degrees = bottom_rotation_in_degrees;
  bool top_limit_switch_detected = false;
  while(!top_limit_switch_detected) {
    linear_1_stepper.step();
    homing_controller.update_sensor();
    if(homing_controller.current_loop > loops){
      Serial.println("Loop detected.");
      loops++;
    }
    int limit_switch_input_top = analogRead(TOP_INPUT_END_STOP);
    top_limit_switch_detected = limit_switch_top.became_turned_on(limit_switch_input_top);
    if(top_limit_switch_detected){
      top_rotation_in_degrees = homing_controller.previous_angle;
      Serial.println("Top limit switch detected.");
      Serial.println(String("Top rotation in degrees: ") + String(top_rotation_in_degrees));
      Serial.println(String("Number of loops between bottom and up: ") + String(loops));
    }
  }
  homing_controller.top_angle = top_rotation_in_degrees;
  homing_controller.loops = loops;
  Serial.println(String("Homing of linear actuator finished."));
  return homing_controller;
}

void setup() {
  Serial2.begin(9600);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Start...");
  
  // actuators
  linear_1_stepper.setup();
  angular_1_stepper.setup();
  angular_2_stepper.setup();
  angular_3_stepper.setup();
  pinMode(BOTTOM_INPUT_END_STOP, INPUT);
  pinMode(TOP_INPUT_END_STOP, INPUT);
  
  // sensors
  linear_1_sensor.init();
  angular_1_sensor.init();
  angular_2_sensor.init();
  angular_3_sensor.init();
 
}

continious_state current_state;
continious_state target_state;



uint32_t iteration_end_stop_ignore = 0;
uint32_t sensor_read_steps = 10;
uint32_t show_log_time = 10;
bool show_log = false;

LinearController linear_1_controller;

int iteration = 0;
int max_iteration = 1;

bool in_target_linear_1;

void loop() {
  uint32_t delay_in_loop = 10;
  if(!linear_1_controller.is_initialized) {
    linear_1_controller = home_linear();
    /* WARNING, CHANGE THIS
     * TODO: this is only a mock
     */
    //target_state.linear_1  = 250.0;
    target_state.angular_1 = 90.0;
    target_state.angular_2 = 0.0;
    target_state.angular_3 = 0.0;
    
    //TODO: REMOVE THIS!!!!
    //linear_1_controller.is_initialized = true;
    
  }
  
  //current_state.linear_1 = linear_1_controller.height_value;
  current_state.angular_1 = angular_1_controller.angle_value;
  current_state.angular_2 = angular_2_controller.angle_value;
  current_state.angular_3 = angular_3_controller.angle_value;
  
/*
  Serial.println("Current State is. Angle 1: " + String(current_state.angular_1) 
        + "| Angle 2: " + String(current_state.angular_2) 
        + "| Angle 3: " + String(current_state.angular_3)
        + "| Linear 1: " + String(current_state.linear_1));
*/        
  
  communication.communication_loop(current_state, target_state);
  if(sensor_read_steps > 0){
    sensor_read_steps --;
  }
  
  delay_in_loop += angular_1_controller.control(target_state.angular_1);
  delay_in_loop += angular_2_controller.control(target_state.angular_2);
  delay_in_loop += angular_3_controller.control(target_state.angular_3);
  in_target_linear_1 = linear_1_controller.control(target_state.linear_1);
    
  if(sensor_read_steps == 0){
    sensor_read_steps = 5;
  }

  if(show_log_time > 0){
    show_log_time--;
  }
  if(show_log_time == 0){
    //Serial.println(String("Distance in mm: ") + String(linear_1_controller.distance_in_mm()));
    //Serial.println(String("Delay in loop: ") + String(delay_in_loop));    
    show_log_time = 1000;
  }
  delayMicroseconds(delay_in_loop * 1);
};
