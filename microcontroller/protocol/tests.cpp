#include "protocol.hpp"
#include <cassert>
#include <iostream>
using namespace protocol;

void print_message(Message message) {
    for(int i = 0; i < message.get_message_length();i++){
        std::cout << "0x" << std::hex << (int)(message.message[i] & 0xFF) << " ";
    }
    std::cout << std::endl;
}

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
    Message home_message = Message::make_homing_message();
    home_message_generation_test(home_message);
    home_messag_parsing_test(home_message);
    home_messag_parsing_with_corrupted_checksum_test(home_message);
    home_messag_parsing_with_corrupted_end_flag_test(home_message);
}

void home_response_message_test() {
    char* message = Message::make_homing_response_message().message;
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
    assert(parsing_result4.get_is_parsed());
    std::cout << "SUCCESS -- RESPONSE MESSAGE PARSING" << std::endl;
}

void configure_message_test() {
    Message message = Message::make_configure_message(true, 6, 7);
    char expected_message[7] = { (char)0xAA, (char)0x03, (char)0x01, (char)0x06, (char)0x07, (char)(0x00 ^ 0x03 ^ 0x01 ^ 0x06 ^ 0x07), (char)0xFF};
    assert(message.message[0] == expected_message[0]);
    assert(message.message[1] == expected_message[1]);
    assert(message.message[2] == expected_message[2]);
    assert(message.message[3] == expected_message[3]);
    assert(message.message[4] == expected_message[4]);
    assert(message.message[5] == expected_message[5]);
    assert(message.message[6] == expected_message[6]);
    std::cout << "SUCCESS -- CONFIGURE MESSAGE CREATION" << std::endl;
}


void home_creation_message_test() {
    Message message = Message::make_homing_message();
    char expected_message[7] = { (char)0xAA, (char)0x01, (char)(0x00 ^ 0x01), (char)0xFF};
    assert(message.message[0] == expected_message[0]);
    assert(message.message[1] == expected_message[1]);
    assert(message.message[2] == expected_message[2]);
    assert(message.message[3] == expected_message[3]);
    assert(message.get_message_length() == 4);
    std::cout << "SUCCESS -- HOME MESSAGE CREATION" << std::endl;
}


void return_home_creation_message_test() {
    Message message = Message::make_homing_response_message();
    char expected_message[4] = { (char)0xAA, (char)0x02, (char)0x2, (char)0xFF};
    assert(message.message[0] == expected_message[0]);
    assert(message.message[1] == expected_message[1]);
    assert(message.message[2] == expected_message[2]);
    assert(message.message[3] == expected_message[3]);
    assert(message.get_message_length() == 4);
    std::cout << "SUCCESS -- RETURN HOMING RESPONSE MESSAGE CREATION" << std::endl;
}


void return_homing_state_creation_message_test() {
    Message message = Message::make_homing_state_response_message(0x12);
    char expected_message[5] = { (char)0xAA, (char)0x06, (char)0x12, (char) 0x14, (char) 0xFF };
    assert(message.message[0] == expected_message[0]);
    assert(message.message[1] == expected_message[1]);
    assert(message.message[2] == expected_message[2]);
    assert(message.message[3] == expected_message[3]);
    assert(message.message[4] == expected_message[4]);
    assert(message.get_message_length() == 5);
    std::cout << "SUCCESS -- RETURN HOME STATE MESSAGE CREATION" << std::endl;
}


void return_homing_state_creation_message_with_flags_in_data_test() {
    Message message = Message::make_homing_state_response_message(0xAA);
    char expected_message[6] = { (char)0xAA, (char)0x06, (char)0xBB, (char)0xAA, (char) 0x17, (char) 0xFF };
    assert(message.message[0] == expected_message[0]);
    assert(message.message[1] == expected_message[1]);
    assert(message.message[2] == expected_message[2]);
    assert(message.message[3] == expected_message[3]);
    assert(message.message[4] == expected_message[4]);
    assert(message.message[5] == expected_message[5]);
    assert(message.get_message_length() == 6);

    ParsingResult parse_result;
    Parser parser;
    for(uint32_t idx = 0; idx < message.get_message_length(); idx++){
        parse_result = parser.parse_byte(message.message[idx]);
    }
    assert(parse_result.get_is_parsed());
    assert(parse_result.get_message().get_message_type() == message.get_message_type());
    for(uint32_t idx = 0; idx < message.get_message_length(); idx++){
        assert(message.message[idx] == parse_result.get_message().message[idx]);
    }

    std::cout << "SUCCESS -- RETURN HOME STATE MESSAGE CREATION" << std::endl;
}


void protocol_test() {
    Parser parser;
    ParsingResult parse_result;

    Message homing_message = Message::make_homing_response_message();
    //print_message(homing_message);
    for(uint32_t idx = 0; idx < homing_message.get_message_length(); idx++){
        parse_result = parser.parse_byte(homing_message.message[idx]);
        //std::cout  << (int) homing_message.message[idx] << ": " << static_cast<int>(parse_result.get_state()) << std::endl;
    }
    assert(parse_result.get_is_parsed());
    assert(parse_result.get_message().get_message_type() == homing_message.get_message_type());

    Message homing_state_message = Message::make_homing_state_response_message(0x12);
    //print_message(homing_state_message);
    char expected_message[5] = { (char)0xAA, (char)0x06, (char)0x12, (char) 0x14, (char) 0xFF };
    for(uint32_t idx = 0; idx < homing_state_message.get_message_length(); idx++){
        parse_result = parser.parse_byte(homing_state_message.message[idx]);
        //std::cout  << (int) homing_state_message.message[idx] << ": " << static_cast<int>(parse_result.get_state()) << std::endl;
    }
    assert(parse_result.get_is_parsed());
    assert(parse_result.get_message().get_message_type() == homing_state_message.get_message_type());
    for(uint32_t idx = 0; idx < homing_state_message.get_message_length(); idx++){
        assert(homing_state_message.message[idx] == parse_result.get_message().message[idx]);
    }
    std::cout << "SUCCESS -- PROTOCOL MESSAGE SEQUENCE TEST" << std::endl;
}

int main(int argc, char **argv) {
    home_creation_message_test();
    home_message_tests();
    home_response_message_test();
    return_home_creation_message_test();
    configure_message_test();
    return_homing_state_creation_message_test();
    protocol_test();
    return_homing_state_creation_message_with_flags_in_data_test();
    return 0;
}