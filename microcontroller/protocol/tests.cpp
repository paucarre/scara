#include "protocol.hpp"
#include <cassert>
#include <iostream>
using namespace protocol;

void message_generation_test(MessageType message_type) {
    const char data[0] = {};
    char message[message_type.get_message_size()] = {0};
    MessageFactory::write_message_data(message_type, data, message);
    assert(message[0] == Parser::START_FLAG);
    assert(message[1] == message_type.get_label());
    assert(message[2] == 0x00 ^ message_type.get_label());
    assert(message[3] == Parser::END_FLAG);
    assert(sizeof(message)/sizeof(*message) == 4);
    std::cout << "SUCCESS -- MESSAGE GENERATION" << std::endl;
}

void messag_parsing_test(MessageType message_type) {
    char message[message_type.get_message_size()] = {0};
    const char data[0] = {};
    MessageFactory::write_message_data(message_type, data, message);
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(message[2]);
    assert(parser.get_state() == ParsingState::FINDING_END_FLAG);
    ParsingResult parsing_result4 = parser.parse_byte(message[3]);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    assert(parsing_result4.get_message().get_message_type().get_label() != UNDEFINED_MESSAGE_TYPE.get_label());
    Message message_parsed = parsing_result4.get_message();
    assert(message_parsed.get_message_type().get_label() == message_type.get_label());
    assert(parsing_result4.get_is_parsed());
    std::cout << "SUCCESS -- MESSAGE PARSING" << std::endl;
}

void messag_parsing_with_corrupted_checksum_test(MessageType message_type) {
    char message[message_type.get_message_size()] = {0};
    const char data[0] = {};
    MessageFactory::write_message_data(message_type, data, message);
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(0xFF);
    assert(parsing_result3.get_parsing_error() == ParsingError::CHECKSUM_VALIDATION_ERROR);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    std::cout << "SUCCESS -- MESSAGE PARSING WITH CORRUPTED CHECKSUM" << std::endl;

}

void messag_parsing_with_corrupted_end_flag_test(MessageType message_type) {
    char message[message_type.get_message_size()] = {0};
    const char data[0] = {};
    MessageFactory::write_message_data(message_type, data, message);
    Parser parser;
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    ParsingResult parsing_result1 = parser.parse_byte(message[0]);
    assert(parser.get_state() == ParsingState::FINDING_MESSAGE_LABEL);
    ParsingResult parsing_result2 = parser.parse_byte(message[1]);
    assert(parser.get_state() == ParsingState::PARSING_MESSAGE);
    ParsingResult parsing_result3 = parser.parse_byte(message[2]);
    assert(parser.get_state() == ParsingState::FINDING_END_FLAG);
    ParsingResult parsing_result4 = parser.parse_byte(0xAA);
    assert(parsing_result4.get_parsing_error() == ParsingError::END_FLAG_NOT_FOUND);
    assert(parser.get_state() == ParsingState::FINDING_START_FLAG);
    std::cout << "SUCCESS -- MESSAGE PARSING WITH CORRUPTED END FLAG" << std::endl;
}

void message_tests(MessageType message_type) {
    message_generation_test(message_type);
    messag_parsing_test(message_type);
    messag_parsing_with_corrupted_checksum_test(message_type);
    messag_parsing_with_corrupted_end_flag_test(message_type);
}

int main(int argc, char **argv) {
    message_tests(HOME_MESSAGE_TYPE);
    message_tests(RESPONSE_MESSAGE_TYPE);
    return 0;
}