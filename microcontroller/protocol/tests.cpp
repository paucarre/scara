#include "protocol.hpp"
#include <cassert>
#include <iostream>
using namespace protocol;

void home_message_generation_test(Message message) {
    MessageType message_type = message.get_message_type();
    assert(message.message[0] == Parser::START_FLAG);
    assert(message.message[1] == message_type.get_label());
    uint8_t data_length = message_type.get_data_length();
    assert(message.message[2 + data_length] == message_type.get_label());
    assert(message.message[3 + data_length] == Parser::END_FLAG);
    std::cout << "SUCCESS -- HOME MESSAGE GENERATION" << std::endl;
}

void home_messag_parsing_test(Message message) {
    MessageType message_type = message.get_message_type();
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message.message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message.message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(message.message[2]);
    assert(parser.get_state() == ParsingState::FINDING_END_FLAG);
    ParsingResult parsing_result4 = parser.parse_byte(message.message[3]);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    assert(parsing_result4.get_message().get_message_type().get_label() != UNDEFINED_MESSAGE_TYPE.get_label());
    Message message_parsed = parsing_result4.get_message();
    assert(message_parsed.get_message_type().get_label() == message_type.get_label());
    assert(parsing_result4.get_is_parsed());
    std::cout << "SUCCESS -- HOME MESSAGE PARSING" << std::endl;
}

void home_messag_parsing_with_corrupted_checksum_test(Message message) {
    MessageType message_type = message.get_message_type();
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message.message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message.message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(0xFF);
    assert(parsing_result3.get_parsing_error() == ParsingError::CHECKSUM_VALIDATION_ERROR);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    std::cout << "SUCCESS -- HOME MESSAGE PARSING WITH CORRUPTED CHECKSUM" << std::endl;

}

void home_messag_parsing_with_corrupted_end_flag_test(Message message) {
    MessageType message_type = message.get_message_type();
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message.message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message.message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(message.message[2]);
    assert(parser.get_state() == ParsingState::FINDING_END_FLAG);
    ParsingResult parsing_result4 = parser.parse_byte(0xAA);
    assert(parsing_result4.get_parsing_error() == ParsingError::END_FLAG_NOT_FOUND);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    std::cout << "SUCCESS -- HOME MESSAGE PARSING WITH CORRUPTED END FLAG" << std::endl;
}

void home_message_tests() {
    Message home_message = Message::make_home_message();
    home_message_generation_test(home_message);
    home_messag_parsing_test(home_message);
    home_messag_parsing_with_corrupted_checksum_test(home_message);
    home_messag_parsing_with_corrupted_end_flag_test(home_message);
}

void response_message_test() {
    char* message = Message::make_response_message(Message::make_home_message()).message;
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(message[2]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result4 = parser.parse_byte(message[3]);
    assert(parser.get_state() == ParsingState::FINDING_END_FLAG);
    ParsingResult parsing_result5 = parser.parse_byte(message[4]);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    assert(parsing_result5.get_is_parsed());
    std::cout << "SUCCESS -- RESPONSE MESSAGE PARSING" << std::endl;
}

int main(int argc, char **argv) {
    home_message_tests();
    response_message_test();
    return 0;
}