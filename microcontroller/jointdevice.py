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


    def try_to_get_response(self, MAX_ATTEMTS=20):
        current_attempt = 0
        while current_attempt < MAX_ATTEMTS:
            if self.serial_handler.in_waiting > 0:
                received_byte = self.serial_handler.read()
                parsing_result = self.parser.parse_byte(received_byte)
                if parsing_result.is_parsed():
                    message = parsing_result.get_message()
                    #TODO: try to find a mapping between request and resonse with the right message type
                    #if message.get_message_type() == protocol.RESPONSE_MESSAGE_TYPE:
                    return Success(message)
            else:
                time.sleep(0.2)
                current_attempt += 1
        return Failure("Failed all attempts to receive the response message")

    def send_message(self, message):
        message_bytes = message.get_bytes()
        try:
            bytes_written = self.serial_handler.write(message_bytes)
            self.serial_handler.flush()
            if bytes_written == len(message_bytes):
                return Success(f'Message successfully sent')
            else:
                return Failure(f'Error writting message bytes')
        except Exception as error:
            return Failure(error)

    def configure(self):
        configure_message = protocol.Message.make_configure_message(self.dir_high_is_clockwise, self.dir_pin, self.step_pin)
        configure_message_result = self.send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: self.try_to_get_response())
        return configure_message_result

    def home(self):
        home_message = protocol.Message.make_homing_message()
        home_result = self.send_message(home_message)
        time.sleep(0.5)
        home_result = home_result.bind(lambda message: self.try_to_get_response())
        return home_result

    def get_home_state(self):
        homing_sate_message = protocol.Message.make_homing_state_message()
        home_state_result = self.send_message(homing_sate_message)
        home_state_result = home_state_result.bind(lambda message: self.try_to_get_response())
        home_state_result = home_state_result.value_or(None)
        if home_state_result is not None:
            home_state_result = home_state_result.get_data()
        return home_state_result

    def close(self):
        self.serial_handler.close()


angular_joint_1_device = JointDevice('/dev/ttyS6', False, 27, 26)
angular_joint_2_device = JointDevice('/dev/ttyS8', False, 27, 26)
angular_joint_3_device = JointDevice('/dev/ttyS', False, 27, 26)

#joints = [angular_joint_2_device]

#for joint in joints:
    #result = joint.configure()
    #print(result)
    #time.sleep(0.2)
    #result = joint.home()
    #print(result)

result = angular_joint_2_device.home().value_or(None).get_bytes()
print('HOME: ', result)
time.sleep(1)
result = angular_joint_2_device.get_home_state()
print('HOME STATE: ', result)
angular_joint_2_device.close()