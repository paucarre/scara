import serial
import protocol
import time
import math
from returns.result import Failure, ResultE, Success
from multiprocessing import Process
import logging

class ControllerMinmaxConfiguration():

    def __init__(self, minimum_steps, maximum_steps):
        self.minimum_steps = minimum_steps
        self.maximum_steps = maximum_steps

    def __repr__(self):
        return str(self)

    def __str__(self):
        return str(self.__dict__)


class ControllerConfiguration():

    def __init__(self, minimum_steps, maximum_steps, max_speed_steps_per_second, max_acceleration_steps_per_second_squared):
        self.minimum_steps = minimum_steps
        self.maximum_steps = maximum_steps
        self.max_speed_steps_per_second = max_speed_steps_per_second
        self.max_acceleration_steps_per_second_squared = max_acceleration_steps_per_second_squared

    def __repr__(self):
        return str(self)

    def __str__(self):
        return str(self.__dict__)

class JointDevice():

    def __init__(self, label, actuator_type, serial_name, dir_high_is_clockwise, dir_pin, step_pin, homing_offset, min_steps, max_steps, 
            max_speed_steps_per_second, max_acceleration_steps_per_second_squared, baud_rate=9600):
        self.label = label
        self.parser = protocol.Parser()
        self.actuator_type = actuator_type
        self.serial_name = serial_name
        self.baud_rate = baud_rate
        self.dir_high_is_clockwise = dir_high_is_clockwise
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.homing_offset = homing_offset
        self.min_steps = min_steps
        self.max_steps = max_steps
        self.max_speed_steps_per_second = max_speed_steps_per_second
        self.max_acceleration_steps_per_second_squared = max_acceleration_steps_per_second_squared

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

    def configure(self):
        configure_message = protocol.Message.make_configure_message(
            self.dir_high_is_clockwise, self.dir_pin, self.step_pin, self.homing_offset, self.actuator_type)
        configure_message_result = self._try_to_send_message(configure_message)
        print("send config result: ", configure_message_result)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.CONFIGURE_RESPONSE_MESSAGE_TYPE))
        print("send config response", configure_message_result)
        configure_message_result = configure_message_result.map(lambda message: \
            message.get_data()).value_or(None)
        print(configure_message_result)
        self.configure_control_configuration(self.min_steps, self.max_steps,
            self.max_speed_steps_per_second, self.max_acceleration_steps_per_second_squared)
        return configure_message_result


    def configure_control_configuration(self, minimum_steps, maximum_steps, max_speed_steps_per_second, max_acceleration_steps_per_second_squared):
        configure_message = protocol.Message.make_set_control_configuration_message(minimum_steps, maximum_steps,
            max_speed_steps_per_second, max_acceleration_steps_per_second_squared)
        configure_message_result = self._try_to_send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.SET_CONTROL_CONFIGURATION_RESPONSE_MESSAGE_TYPE))
        configure_message_result = configure_message_result.map(lambda message: \
            ControllerConfiguration ( \
                protocol.Message.make_int32_from_four_bytes(message.get_data()[0], message.get_data()[1], message.get_data()[2], message.get_data()[3]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[4], message.get_data()[5], message.get_data()[6], message.get_data()[7]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[8], message.get_data()[9], message.get_data()[10], message.get_data()[11]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[12], message.get_data()[13], message.get_data()[14], message.get_data()[15])
            )).value_or(None)
        return configure_message_result

    def get_configure_control_configuration(self):
        configure_message = protocol.Message.make_get_control_configuration_message()
        configure_message_result = self._try_to_send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_CONTROL_CONFIGURATION_RESPONSE_MESSAGE_TYPE))
        configure_message_result = configure_message_result.map(lambda message: \
            ControllerConfiguration ( \
                protocol.Message.make_int32_from_four_bytes(message.get_data()[0], message.get_data()[1], message.get_data()[2], message.get_data()[3]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[4], message.get_data()[5], message.get_data()[6], message.get_data()[7]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[8], message.get_data()[9], message.get_data()[10], message.get_data()[11]),
                protocol.Message.make_int32_from_four_bytes(message.get_data()[12], message.get_data()[13], message.get_data()[14], message.get_data()[15])
            )).value_or(None)
        return configure_message_result

    '''
    def configure_controller(self, error_constant, max_microseconds_delay):
        configure_message = protocol.Message.make_set_control_configuration_message(error_constant, max_microseconds_delay)
        configure_message_result = self._try_to_send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.SET_CONTROL_CONFIGURATION_RESPONSE_MESSAGE_TYPE))
        configure_message_result = configure_message_result.map(lambda message: \
            ControllerConfiguration ( \
                protocol.Message.make_uint16_from_two_bytes(message.get_data()[0], message.get_data()[1]),
                protocol.Message.make_uint16_from_two_bytes(message.get_data()[2], message.get_data()[3]),
            )).value_or(None)
        return configure_message_result
    '''

    def get_controller_configuration(self):
        configure_message = protocol.Message.make_get_control_configuration_message()
        configure_message_result = self._try_to_send_message(configure_message)
        configure_message_result = configure_message_result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_CONTROL_CONFIGURATION_RESPONSE_MESSAGE_TYPE))
        configure_message_result = configure_message_result.map(lambda message: \
            ControllerConfiguration ( \
                protocol.Message.make_uint16_from_two_bytes(message.get_data()[0], message.get_data()[1]),
                protocol.Message.make_uint16_from_two_bytes(message.get_data()[2], message.get_data()[3]),
            )).value_or(None)
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
        configure_result = self.configure_control_configuration(5000, 500, 10000, 50)
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
        result = self._try_to_send_message(message)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.SET_TARGET_STEPS_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: message.get_data()).value_or(None)
        return result

    def get_target_steps(self):
        message = protocol.Message.make_get_target_steps_message()
        result = self._try_to_send_message(message)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_TARGET_STEPS_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: \
            protocol.Message.make_int32_from_four_bytes(message.get_data()[0], message.get_data()[1], message.get_data()[2], message.get_data()[3])).value_or(None)
        return result

    def get_steps(self):
        message = protocol.Message.make_get_steps_message()
        result = self._try_to_send_message(message)
        result = result.bind(lambda message: \
            self._try_to_get_response(protocol.GET_STEPS_RESPONSE_MESSAGE_TYPE))
        result = result.map(lambda message: \
            protocol.Message.make_int32_from_four_bytes(message.get_data()[0], message.get_data()[1], message.get_data()[2], message.get_data()[3])).value_or(None)
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
            self.logger.debug(f'Homing Sate: {result}')

    def configure_until_finished(self):
        is_finished = lambda result: ( (result[0] == 1 and self.dir_high_is_clockwise) or (result[0] == 0 and not self.dir_high_is_clockwise) ) and \
            (result[1] == self.dir_pin) and \
            (result[2] == self.step_pin) and \
            (protocol.Message.make_int16_from_two_bytes(result[3], result[4]) == self.homing_offset) and \
            (protocol.ActuatorType.from_index(result[5]) == self.actuator_type)
        result = self.configure()
        result = self.configure()
        while not is_finished(result):
            time.sleep(0.1)
            self.logger.warn(f'Trying to configure joint device: {self}')
            result = self.get_configuration()
        return result

    def move_to_target_until_is_reached(self, target_steps):
        #print(f"---- {target_steps} ----")
        result = self.set_target_steps(target_steps)
        if target_steps > self.max_steps:
            target_steps = self.max_steps
        elif target_steps < self.min_steps:
            target_steps = self.min_steps
        #print(result)
        steps = self.get_steps()        
        while steps is None or steps != target_steps:
            print('target steps: ', self.get_target_steps())
            steps = self.get_steps()
            print("STEPS: ", steps)


if __name__ == '__main__':
    #joint = JointDevice('Linear 0', protocol.ActuatorType.LINEAR, '/dev/ttyS5', True, 27, 26, 0, 1000, 51000)
    joint = JointDevice('Angular 0', protocol.ActuatorType.ROTARY, '/dev/ttyUSB0', 
        True, 15, 2, -425, -26000, 26000, 1000000, 5)
    open_result = joint.open()
    test = joint.get_configure_control_configuration()
    print(test)
    #joint.close()
    #open_result = joint.open()
    #print("Device opened: ", open_result )
    configuration_result = joint.configure()
    #configuration_result.map(lambda result: print(result.get_data()))
    #print("Device configuration: ", configuration_result)
    homing_result = joint.home_until_finished()
    print("Device homed: ", homing_result)
    #max_min_steps = joint.configure_min_max_steps(1000, 51000)
    #print("Device set to min and max steps", max_min_steps)
    result = joint.configure_control_configuration(joint.min_steps, joint.max_steps,
        joint.max_speed_steps_per_second, joint.max_acceleration_steps_per_second_squared)
    print(result)    
    multi = 1
    while True:
        joint.move_to_target_until_is_reached(multi * 5000)
        multi = multi * -1
    #print("Device homed")
    #joint.move_to_target_until_is_reached(+26000)
    #joint.move_to_target_until_is_reached(10000)
    joint.close()
    print("Device closed")
