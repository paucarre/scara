import serial
import protocol
import time
from returns.result import Failure, ResultE, Success

class JointDevice():

    def __init__(self, serial_name, dir_high_is_clockwise, dir_pin, step_pin, baud_rate=9600):
        self.parser = protocol.Parser()
        self.serial_name = serial_name
        self.baud_rate = baud_rate
        self.dir_high_is_clockwise = dir_high_is_clockwise
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.serial_handler = serial.Serial(self.serial_name, self.baud_rate, timeout=1)


    def _try_to_get_response(self, serial_handler, MAX_ATTEMTS=100):
        current_attempt = 0
        while current_attempt < MAX_ATTEMTS:
            if serial_handler.in_waiting > 0:
                received_byte = serial_handler.read()
                parsing_result = self.parser.parse_byte(received_byte)
                #print(parsing_result)
                if parsing_result.is_parsed():
                    message = parsing_result.get_message()
                    #TODO: try to find a mapping between request and resonse
                    #if message.get_message_type() == protocol.RESPONSE_MESSAGE_TYPE:
                    return Success(message)
            else:
                time.sleep(0.1)
                current_attempt += 1
        return Failure("Failed all attempts to receive the response message")

    def send_message(self, message):
        message_bytes = message.get_bytes()
        try:
            bytes_written = self.serial_handler.write(message_bytes)
            self.serial_handler.flush()
            if bytes_written == len(message_bytes):
                return_message_either = self._try_to_get_response(self.serial_handler)
                return return_message_either.unify(lambda message: Success(f'Message successfully received'))
            else:
                return Failure(f'Error writting message bytes')
        except Exception as error:
            return Failure(error)

    def configure(self):
        configure_message = protocol.Message.make_configure_message(self.dir_high_is_clockwise, self.dir_pin, self.step_pin)
        return self.send_message(configure_message)

    def home(self):
        homing_message = protocol.Message.make_homing_message()
        return self.send_message(homing_message)

    def close(self):
        self.serial_handler.close()


#angular_joint_1_device = JointDevice('/dev/ttyS6', False, 27, 26)
angular_joint_2_device = JointDevice('/dev/ttyS8', False, 27, 26)
#angular_joint_3_device = JointDevice('/dev/ttyS', False, 27, 26)

joints = [angular_joint_2_device]

for joint in joints:
    #result = joint.configure()
    #print(result)
    #time.sleep(0.2)
    result = joint.home()
    print(result)

time.sleep(100)