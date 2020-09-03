import serial
import protocol
import time
from returns.result import Failure, ResultE, Success

class JointDevice():

    def __init__(self, serial_name, baud_rate=9600):
        self.parser = protocol.Parser()
        self.serial_name = serial_name
        self.baud_rate = baud_rate

    def _try_to_get_response(self, serial_handler, MAX_ATTEMTS=10):
        current_attempt = 0
        while current_attempt < MAX_ATTEMTS:
            if serial_handler.inWaiting():
                received_byte = serial_handler.readline()
                parsing_result = self.parser.parse_bytes(received_byte)
                if parsing_result.is_parsed():
                    message = parsing_result.get_message()
                    if message.get_message_type() == protocol.RESPONSE_MESSAGE_TYPE:
                        return Success(message)
                return Failure("Could not parse reponse message")
            else:
                time.sleep(10 / 1000)
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


joint_1_device = JointDevice('/dev/ttyS6')
home_message = protocol.Message.make_home_message()
result = joint_1_device.send_message(home_message)
print(result)
