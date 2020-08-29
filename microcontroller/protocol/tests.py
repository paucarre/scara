import protocol
home_message = protocol.Message.make_home_message()
print(home_message.get_bytes())