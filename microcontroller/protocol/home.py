import serial
import protocol
import time

AXIS_1_SERIAL = '/dev/ttyS6'
SERIAL_BAUDRATE = 9600
MAX_ATTEMTS = 10

home_message = protocol.Message.make_home_message()
message_bytes = home_message.get_bytes()

parser = protocol.Parser()
with serial.Serial(AXIS_1_SERIAL, SERIAL_BAUDRATE, timeout=1) as serial_handler:
    bytes_written = serial_handler.write(message_bytes)
    print(f'Bytes written: {bytes_written}')


    current_attempt = 0
    while current_attempt < MAX_ATTEMTS:
        if serial_handler.inWaiting():
            received_byte = serial_handler.readline()
            parsing_result = parser.parse_bytes(received_byte)
            if parsing_result.is_parsed():
                message = parsing_result.get_message()
                if message.get_message_type() == protocol.RESPONSE_MESSAGE_TYPE:
                    print(f'Received Response')
                else:
                    time.sleep(10 / 1000)
                    current_attempt += 1