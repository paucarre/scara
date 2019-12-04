from kinematics import RobotState

import serial
import sys 
import math

class ArduinoConnector():
    '''
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
    '''
    
    WRITE_TARGET_STATE_HEADER = bytes([0xCF, 0xFF])
    READ_CURRENT_STATE_HEADER = bytes([0xEF, 0xFF])
    READ_TARGET_STATE_HEADER  = bytes([0xDF, 0xFF])

    def __init__(self, device='/dev/ttyACM0'):
        self.serial_connection = serial.Serial(device, 9600, timeout=5)

    def read_until_received_header_received(self, header):
        next_header_index_to_read = 0
        while(next_header_index_to_read < 2):
            byte_received = self.serial_connection.read(size=1)
            if byte_received == b'':
                print("No byte received, returning False", file=sys.stderr)
                return False
            return_value_header_received = byte_received ==  bytes([header[next_header_index_to_read]])
            if return_value_header_received:
                next_header_index_to_read += 1
            elif next_header_index_to_read > 0:
                print(f"Wrong byte received '{byte_received}' instead of {header[next_header_index_to_read]}, returning False", file=sys.stderr)
                return False
        return True

    @staticmethod
    def to_rads(grads):
        return (grads / 360) * (2 * math.pi)
    
    def read_state(self, header):
        self.serial_connection.write(header)
        success = self.read_until_received_header_received(header)
        if success:
            packet = self.serial_connection.read(size=9)
            if(len(packet) == 9):
                linear_1 = packet[0] << 8 | packet[1]
                angular_1 = packet[2] << 8 | packet[3]
                angular_2 = packet[4] << 8 | packet[5]
                angular_3 = packet[6] << 8 | packet[7]
                computed_check = (linear_1 ^ angular_1 ^ angular_2 ^ angular_3) & 0x00FF
                if computed_check == packet[8]:
                    return RobotState(
                        linear_1 = linear_1 / 8.0, 
                        angle_1 = ArduinoConnector.to_rads(angular_1 / 8.0), 
                        angle_2 = ArduinoConnector.to_rads(angular_2 / 8.0),
                        angle_3 = ArduinoConnector.to_rads(angular_3 / 8.0))
                else:
                    print(f"Error checking error code in serial packet. Computed {computed_check} | Received {packet[8]}", 
                        file=sys.stderr)
        return None

    def read_current_state(self):
        return self.read_state(ArduinoConnector.READ_CURRENT_STATE_HEADER)

    def read_target_state(self):
        return self.read_state(ArduinoConnector.READ_TARGET_STATE_HEADER)

    @staticmethod
    def split_uint16_t(data):
        data_high = data >> 8
        data_low = data & 0x00FF
        return data_high, data_low 

    def write_target_state(self, state):
        
        linear_1_sensor_results = int(state[0] * 8)
        angular_1_sensor_results = int(state[1] * 8)
        angular_2_sensor_results = int(state[2] * 8)
        angular_3_sensor_results = int(state[3] * 8)

        linear_1_high, linear_1_low = ArduinoConnector.split_uint16_t(linear_1_sensor_results)
        angular_1_high, angular_1_low = ArduinoConnector.split_uint16_t(angular_1_sensor_results)
        angular_2_high, angular_2_low = ArduinoConnector.split_uint16_t(angular_2_sensor_results)
        angular_3_high, angular_3_low = ArduinoConnector.split_uint16_t(angular_3_sensor_results)

        check = (linear_1_sensor_results ^ angular_1_sensor_results ^ 
            angular_2_sensor_results ^ angular_3_sensor_results) & 0x00FF
        self.serial_connection.write(ArduinoConnector.WRITE_TARGET_STATE_HEADER)
        print([linear_1_high, linear_1_low, angular_1_high, angular_1_low,
            angular_2_high, angular_2_low, angular_3_high, angular_3_low, check])
        packet = bytes([linear_1_high, linear_1_low, angular_1_high, angular_1_low,
            angular_2_high, angular_2_low, angular_3_high, angular_3_low, check])
        self.serial_connection.write(packet)

      
#arduino_connector = ArduinoConnector('/dev/ttyUSB0')
#current_state = arduino_connector.read_current_state()
#print(current_state)

