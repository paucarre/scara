import serial
import protocol

home_message = protocol.Message.make_home_message()
parser = protocol.Parser()
message_bytes = home_message.get_bytes()
print(message_bytes)
parsing_result = parser.parse_bytes(message_bytes)
print(parsing_result.get_state())
print(parsing_result.is_parsed())
print(parsing_result.get_message())
AXIS_1_SERIAL = '/dev/ttyS6'
SERIAL_BAUDRATE = 9600
with serial.Serial(AXIS_1_SERIAL, SERIAL_BAUDRATE, timeout=1) as serial_handler:
    bytes_written = serial_handler.write(message_bytes)
    print(bytes_written)