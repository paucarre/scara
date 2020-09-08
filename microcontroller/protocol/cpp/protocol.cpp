#include "protocol.hpp"
#include <iostream>

namespace protocol {

    char MessageFactory::make_checksum(MessageType message_type, char* data, uint8_t begin, uint8_t end) {
        uint8_t checksum = message_type.get_label();
        for(uint8_t i = begin; i < end; i++){
            checksum = checksum ^ data[i];
        }
        return checksum;
    }

    uint8_t MessageFactory::write_message_data(MessageType message_type, const char* data, char* message_out) {
        uint8_t escaped_bytes = 0;
        message_out[0] = Parser::START_FLAG;
        message_out[1] = message_type.get_label();
        uint8_t i = 2;
        for(; i < message_type.get_data_length() + escaped_bytes + 2;){
            //std::cout << (int) data[i] << std::endl;
            if(Parser::is_flag(data[i])) {
                message_out[i] = Parser::ESCAPE_FLAG;
                escaped_bytes ++;
                i++;
            }
            message_out[i] = data[i - 2];
            i++;
        }
        char checksum = make_checksum(message_type, message_out,  2, message_type.get_data_length() + 3);
        if(Parser::is_flag(data[i])) {
            message_out[i] = Parser::ESCAPE_FLAG;
            escaped_bytes ++;
            i++;
        }
        message_out[i] = checksum;
        message_out[i + 1] = Parser::END_FLAG;
        return escaped_bytes;
    }


    ParsingError Parser::validate_checksum(MessageType message_type, uint8_t parsed_checksum) {
        ParsingError parsing_error = ParsingError::NO_ERROR;
        uint8_t computed_checksum = MessageFactory::make_checksum(message_type, parsed_data, 0, data_index);
        if(parsed_checksum == computed_checksum) {
            //std::cout << "good checksum" << std::endl;
            state = ParsingState::FINDING_END_FLAG;
        } else {
            //std::cout << "bad checksum" << std::endl;
            parsing_error = ParsingError::CHECKSUM_VALIDATION_ERROR;
            state = ParsingState::FINDING_START_FLAG;
        }
        return parsing_error;
    }


    ParsingResult Parser::parse_byte(char data) {
        //std::cout << "data " << (int)data << std::endl;
        ParsingError parsing_error = ParsingError::NO_ERROR;
        switch(state) {
            case ParsingState::FINDING_START_FLAG:
                {
                    if(data == START_FLAG) {
                        state = ParsingState::FINDING_MESSAGE_LABEL;
                    } else {
                        parsing_error = ParsingError::START_FLAG_NOT_FOUND;
                    }
                    break;
                }
            case ParsingState::FINDING_MESSAGE_LABEL:
                {
                    message_type = UNDEFINED_MESSAGE_TYPE;
                    for(uint8_t i = 0;(message_type.get_label() == UNDEFINED_MESSAGE_TYPE.get_label()) && i < NUMBER_OF_MESSAGES;++i){
                        if(MESSAGES[i].get_label() == data) {
                            message_type = MESSAGES[i];
                        }
                    }
                    if(message_type.get_label() == UNDEFINED_MESSAGE_TYPE.get_label()) {
                        parsing_error = ParsingError::MESSAGE_LABEL_NOT_FOUND;
                        state = ParsingState::FINDING_START_FLAG;
                    } else {
                        state = ParsingState::PARSING_MESSAGE;
                        data_index = 0;
                        //parsed_data[data_index] = data;
                        //std::cout << "parsed data " << (int)data << std::endl;
                        //data_index++;
                        previous_data_was_escaped = false;
                    }
                    break;
                }
            case ParsingState::PARSING_MESSAGE:
                {
                    if(data_index == message_type.get_data_length()){
                         //std::cout << "validating checksum  " << std::endl;
                        parsing_error = validate_checksum(message_type, data);
                    } else {
                        if(previous_data_was_escaped){
                            parsed_data[data_index++] = data;
                            //std::cout << "parsed data " << (int)data << std::endl;
                            previous_data_was_escaped = false;
                        } else {
                            if(data == ESCAPE_FLAG){
                                //std::cout << " ESCAPE FOUND "  << std::endl;
                                previous_data_was_escaped = true;
                            } else {
                                parsed_data[data_index++] = data;
                                //std::cout << "parsed data " << (int)data << std::endl;
                                previous_data_was_escaped = false;
                            }
                        }
                    }
                    break;
                }
            case ParsingState::FINDING_END_FLAG:
                {
                    state = ParsingState::FINDING_START_FLAG;
                    if(data == END_FLAG) {
                        Message message(message_type, parsed_data);
                        //std::cout << "Final: " << std::endl;
                        for(int i = 0;i < data_index;i++){
                            //std::cout << (int) parsed_data[i];
                        }
                        //std::cout << std::endl;
                        return ParsingResult(state, parsing_error, message, true);
                    } else {
                        parsing_error = ParsingError::END_FLAG_NOT_FOUND;
                    }
                    break;
                }

        }
        return ParsingResult(state, parsing_error, Message(UNDEFINED_MESSAGE_TYPE), false);
    }

    Message::Message(MessageType message_type_ ): message_type(message_type_) {
        const char empty_data[0] = {};
        protocol::MessageFactory::write_message_data(message_type, empty_data, message);
    }

    Message::Message(MessageType message_type_, const char* data_): message_type(message_type_) {
        escaped_bytes = protocol::MessageFactory::write_message_data(message_type, data_, message);
    }

    char Message::get_byte_at(uint8_t byte_index) {
        return message[byte_index];
    }

    const char* Message::get_bytes() {
        return message;
    }

}
