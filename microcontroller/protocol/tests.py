import serial
import protocol

home_message = protocol.Message.make_home_message()
parser = protocol.Parser()

empty_message = b''
parsing_result = parser.parse_bytes(empty_message)
assert parsing_result.get_message() is None, 'Parsing an empy byte array provides no parsing result'

message_bytes = home_message.get_bytes()
assert message_bytes == b'\xaa\x01\x01\xff', 'Home message is properly serialized'

parsing_result = parser.parse_bytes(message_bytes)
assert parsing_result.get_state() == protocol.ParsingState.FINDING_START_FLAG, 'Once a message has been successfully parsed, it goes right to the starting state'
assert parsing_result.is_parsed(), 'A properly seralized message should be able to be parsed'
assert parsing_result.get_message().get_bytes() == message_bytes, 'The parsed message should be the one requested to be parsed'

assert parsing_result.get_message().get_message_type() == protocol.HOME_MESSAGE_TYPE, 'The message type of parsing a home message should be home type'

#!AXIS_1_SERIAL = '/dev/ttyS6'
#~SERIAL_BAUDRATE = 9600
#with serial.Serial(AXIS_1_SERIAL, SERIAL_BAUDRATE, timeout=1) as serial_handler:
#    bytes_written = serial_handler.write(message_bytes)
#    print(bytes_written)