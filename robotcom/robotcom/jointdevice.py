import serial
import protocol
import time
import math
from returns.result import Failure, ResultE, Success
from multiprocessing import Process

class JointDevice():

    def __init__(self, actuator_type, serial_name, dir_high_is_clockwise, dir_pin, step_pin, homing_offset, baud_rate=9600):
        self.parser = protocol.Parser()
        self.actuator_type = actuator_type
        self.serial_name = serial_name
        self.baud_rate = baud_rate
        self.dir_high_is_clockwise = dir_high_is_clockwise
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.homing_offset = homing_offset

    def _try_to_get_response(self, expected_response_type, MAX_ATTEMTS=20):
        current_attempt = 0
        while current_attempt < MAX_ATTEMTS:
            if self.serial_handler.in_waiting > 0:
                received_byte = self.serial_handler.read()
                #print(received_byte)
                parsing_result = self.parser.parse_byte(received_byte)
                #print(parsing_result.get_state())
                if parsing_result.is_parsed():
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

    def configure(self):
        configure_message = protocol.Message.make_configure_message(self.dir_high_is_clockwise, self.dir_pin, self.step_pin, self.homing_offset, self.actuator_type)
        configure_message_result = self._try_to_send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.CONFIGURE_RESPONSE_MESSAGE_TYPE))
        configure_message_result = configure_message_result.map(lambda message: \
            message.get_data()).value_or(None)
        return configure_message_result

    def get_configuration(self):
        message = protocol.Message.make_get_configuration_message()
        result = self._try_to_send_message(message)
        #print(result)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_CONFIGURATION_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: \
            message.get_data()).value_or(None)
        return result

    def home(self):
        home_message = protocol.Message.make_homing_message()
        home_result = self._try_to_send_message(home_message)
        home_result = home_result.bind(lambda message: \
            self._try_to_get_response(protocol.HOME_RESPONSE_MESSAGE_TYPE))
        home_result = home_result.map(lambda message: \
            message.get_data()).value_or(None)
        return home_result

    def get_home_state(self):
        homing_sate_message = protocol.Message.make_homing_state_message()
        home_state_result = self._try_to_send_message(homing_sate_message)
        home_state_result = home_state_result.bind(lambda message: \
            self._try_to_get_response(protocol.HOMING_STATE_RESPONSE_MESSAGE_TYPE))
        home_state_result = home_state_result.map(lambda message: \
            protocol.HomingState.from_index(message.get_data())).value_or(None)
        return home_state_result

    def set_target_steps(self, steps):
        message = protocol.Message.make_set_target_steps_message(steps)
        #print(message.get_message_bytes())
        result = self._try_to_send_message(message)
        #print(result)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.SET_TARGET_STEPS_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: message.get_data()).value_or(None)
        return result

    def get_steps(self):
        message = protocol.Message.make_get_steps_message()
        result = self._try_to_send_message(message)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_STEPS_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: \
            protocol.Message.make_int32_from_four_bytes(message.get_data()[0], message.get_data()[1], message.get_data()[2], message.get_data()[3])).value_or(None)
        if result is None:
            self.close()
            self.open()
        return result

    def open(self):
        self.serial_handler = serial.Serial(self.serial_name, self.baud_rate, timeout=1)
        return self

    def close(self):
        self.serial_handler.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, type, value, traceback):
        self.close()

    def home_until_finished(self):
        result = self.home()
        #print(result)
        time.sleep(0.5)
        result = protocol.HomingState.HOMING_NOT_STARTED
        while result != protocol.HomingState.HOMING_FINISHED:
            result = self.get_home_state()
            print(result)

    def configure_until_finished(self):
        is_finished = lambda result: ( (result[0] == 1 and self.dir_high_is_clockwise) or (result[0] == 0 and not self.dir_high_is_clockwise) ) and \
            (result[1] == self.dir_pin) and \
            (result[2] == self.step_pin) and \
            (protocol.Message.make_int16_from_two_bytes(result[3], result[4]) == self.homing_offset) and \
            (protocol.ActuatorType.from_index(result[5]) == self.actuator_type)
        print(self.actuator_type)
        result = self.configure()
        print(result)
        result = self.configure()
        print(result)
        while not is_finished(result):
            time.sleep(0.1)
            result = self.get_configuration()
        return result

    def move_to_target_until_is_reached(self, target_steps):
        #print(f"---- {target_steps} ----")
        result = self.set_target_steps(target_steps)
        #print(result)
        steps = self.get_steps()
        #print(steps)
        while steps is None or steps != target_steps:
            steps = self.get_steps()
            print(steps)