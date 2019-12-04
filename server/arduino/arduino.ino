

typedef struct  {
  uint16_t linear_1;
  uint16_t angular_1;
  uint16_t angular_2;
  uint16_t angular_3;
} state;


class Communication {
  protected:
    static const uint16_t WRITE_TARGET_STATE_HEADER = 0xFFFF;
    static const uint16_t READ_CURRENT_STATE_HEADER = 0xEEFF;
    static const uint16_t READ_TARGET_STATE_HEADER  = 0xDDFF;
    static const uint16_t UNDEFINED_HEADER  = 0x0000;
    
    static const uint8_t HEADER_STATE_NO_BYTE_RECEIVED   = 0x00;
    static const uint8_t HEADER_STATE_HIGH_BYTE_RECEIVED = 0x01;
    static const uint8_t HEADER_STATE_LOW_BYTE_RECEIVED  = 0x02;

    uint8_t compute_check_byte(state& s);
    uint16_t Communication::read_uint16_t();
    void write_target_state();
    void process_command_type_state(uint8_t byte_read);
    void communication_loop();
    void read_state(state s);
    void send_uint16_t(uint16_t data);
    void try_to_read_command_type();
    
  public:
    state target_state;
    state current_state;
    
    uint16_t command_type = UNDEFINED_HEADER;
    uint8_t command_parsing_state = HEADER_STATE_NO_BYTE_RECEIVED;
  
};


uint8_t Communication::compute_check_byte(state& s) {
  return (s.linear_1 ^ s.angular_1 ^ s.angular_2 ^ s.angular_3) & 0x00FF;
}

uint16_t Communication::read_uint16_t() {
  uint8_t data_high = Serial.read();
  uint8_t data_low = Serial.read();
  return data_high << 8 | data_low;
}

void Communication::write_target_state() {
  while(Serial.available() < 9);
  target_state.linear_1 = read_uint16_t();
  target_state.angular_1 = read_uint16_t();
  target_state.angular_2 = read_uint16_t();
  target_state.angular_3 = read_uint16_t();
  uint8_t computed_check = compute_check_byte(target_state);
  uint8_t read_check = Serial.read();
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
  if(Serial.available()){
    uint8_t command_type_byte = Serial.read();
    if(command_type_byte == 0xFF || command_type_byte == 0xEE || command_type_byte == 0xDD) {
      process_command_type_state(command_type_byte);
    } else {
      // error, resetting parsing
      command_type = 0x0000;
      command_parsing_state = Communication::HEADER_STATE_NO_BYTE_RECEIVED;
    }
  }
}

void Communication::send_uint16_t(uint16_t data){
  uint8_t data_high = data >> 8;
  Serial.write(data_high);
  uint8_t data_low = data & 0x00FF;
  Serial.write(data_low);
}

void Communication::read_state(state s){
  send_uint16_t(s.linear_1);
  send_uint16_t(s.angular_1);
  send_uint16_t(s.angular_2);
  send_uint16_t(s.angular_3);
  uint8_t check = compute_check_byte(s);
  Serial.write(check);
}

void setup()
{
  Serial.begin(19200);
}
void loop() {
  delay(2000);
}
void Communication::communication_loop()
{
  try_to_read_command_type();
  switch(command_type){
    case Communication::READ_CURRENT_STATE_HEADER:
      return read_state(current_state);
      break;
    case Communication::READ_TARGET_STATE_HEADER:
      return read_state(target_state);
      break;
    case Communication::WRITE_TARGET_STATE_HEADER:
      write_target_state();
      break;
  }
  if(command_parsing_state == HEADER_STATE_LOW_BYTE_RECEIVED){
    command_type = 0x0000;
    command_parsing_state = HEADER_STATE_NO_BYTE_RECEIVED;
  }
  
}
