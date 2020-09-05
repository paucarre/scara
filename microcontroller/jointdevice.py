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

    def _try_to_get_response(self, serial_handler, MAX_ATTEMTS=30):
        current_attempt = 0
        while current_attempt < MAX_ATTEMTS:
            while serial_handler.inWaiting():
                received_byte = serial_handler.read()
                parsing_result = self.parser.parse_byte(received_byte)
                print(parsing_result)
                if parsing_result.is_parsed():
                    message = parsing_result.get_message()
                    if message.get_message_type() == protocol.RESPONSE_MESSAGE_TYPE:
                        return Success(message)
                #return Failure(f"Could not parse reponse message: {received_byte}")
            else:
                time.sleep(100 / 1000)
                current_attempt += 1
        return Failure("Failed all attempts to receive the response message")

    def send_message(self, message):
        message_bytes = message.get_bytes()
        try:
            with serial.Serial(self.serial_name, self.baud_rate, timeout=1) as serial_handler:
                bytes_written = serial_handler.write(message_bytes)
                if bytes_written == len(message_bytes):
                    return_message_either = self._try_to_get_response(serial_handler)
                    return return_message_either.unify(lambda message: Success(f'Message successfully received'))
                else:
                    return Failure(f'Error writting message bytes')
        except Exception as error:
            return Failure(error)

    def configure(self):
        configure_message = protocol.Message.make_configure_message(self.dir_high_is_clockwise, self.dir_pin, self.step_pin)
        return self.send_message(configure_message)

    def home(self):
        home_message = protocol.Message.make_home_message()
        return self.send_message(home_message)


angular_joint_1_device = JointDevice('/dev/ttyS6', False, 27, 26)
angular_joint_2_device = JointDevice('/dev/ttyS8', False, 27, 26)
angular_joint_3_device = JointDevice('/dev/ttyS9', False, 27, 26)


joints = [angular_joint_1_device, angular_joint_2_device, angular_joint_3_device]

for joint in joints:
    result = joint.configure()
    print(result)
    result = joint.home()
    print(result)
    time.sleep(2)
