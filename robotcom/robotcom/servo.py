import serial
import protocol
import time
import math
from returns.result import Failure, ResultE, Success
from multiprocessing import Process
import logging

class ServoDevice():

    def __init__(self, label, serial_name, baud_rate=9600):
        self.label = label
        self.parser = protocol.Parser()
        self.serial_name = serial_name
        self.baud_rate = baud_rate

        self.logger = logging.getLogger(f'{label} Axis')
        self.logger.setLevel(logging.DEBUG)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)


    def _try_to_get_response(self, expected_response_type, MAX_ATTEMTS=20):
        current_attempt = 0
        received_bytes = []
        while current_attempt < MAX_ATTEMTS:
            if self.serial_handler.in_waiting > 0:
                received_byte = self.serial_handler.read()
                received_bytes.append(received_byte)
                parsing_result = self.parser.parse_byte(received_byte)
                #print(parsing_result.get_state())
                if parsing_result.is_parsed():
                    data_string = ' | '.join([data.hex() for data in received_bytes])
                    self.logger.debug(f'Received bytes: {data_string}')
                    message = parsing_result.get_message()
                    if(message.get_message_type() == expected_response_type):
                        #print(message.get_bytes())
                        return Success(message)
                    else:
                        return Failure(f'Wrong message type returned. Expected {expected_response_type.get_label()} but received {message.get_message_type().get_label()}')
            else:
                time.sleep(0.2)
                current_attempt += 1
        return Failure("Failed all attempts to receive the response message")

    def _try_to_send_message(self, message):
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

    def set_angle(self, angle):
        set_angle_message = protocol.Message.make_set_servo_duty_message(angle)
        set_angle_message = self._try_to_send_message(set_angle_message)
        set_angle_message = set_angle_message.bind(lambda message: \
            self._try_to_get_response(protocol.SET_SERVO_DUTY_RESPONSE_MESSAGE_TYPE))
        set_angle_message = set_angle_message.map(lambda message: \
            message.get_data()).value_or(None)
        return set_angle_message

    def open(self):
        self.serial_handler = serial.Serial(self.serial_name, self.baud_rate, timeout=1)
        return self

    def close(self):
        self.serial_handler.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, type, value, traceback):
        self.close()

if __name__ == '__main__':
    servo_device = ServoDevice('Servo 0', '/dev/ttyUSB0')
    open_result = servo_device.open()
    #test = servo_device.set_angle(0)
    #time.sleep(2)
    #test = servo_device.set_angle(200)
    #time.sleep(2)
    for i in range(50):
        test = servo_device.set_angle(2500)
        time.sleep(0.5)
        test = servo_device.set_angle(2150)
        time.sleep(0.5)

    #test = servo_device.set_angle(500)
    #time.sleep(4)
    #test = servo_device.set_angle(1000)
    #time.sleep(4)

    #test = servo_device.set_angle(10)
    #time.sleep(2)
    #test = servo_device.set_angle(20)
    #time.sleep(2)
    #test = servo_device.set_angle(20)
    #time.sleep(2)
    #test = servo_device.set_angle(200)
    #time.sleep(2)
    #test = servo_device.set_angle(180)
    #time.sleep(2)
    #print(test)
    servo_device.close()
    print("Servo device closed")
