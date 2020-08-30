import protocol
home_message = protocol.Message.make_home_message()
parser = protocol.Parser()
message_bytes = home_message.get_bytes()
print(message_bytes)
parsing_result = parser.parse_bytes(message_bytes)
print(parsing_result.get_state())
print(parsing_result.is_parsed())