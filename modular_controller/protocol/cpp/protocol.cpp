#include "protocol.hpp"

namespace protocol {

    const uint8_t MessageType::get_message_size() {
        // checksum, start, end and label
        return data_lenght + 4;
    }

    const uint8_t MessageType::get_body_size() {
        // label
        return (this->data_lenght) + 1;
    }

    uint8_t MessageFactory::make_checksum(uint8_t* data, uint8_t lenght) {
        uint8_t checksum = 0;
        for(uint8_t i = 0; i < lenght; i++){
            checksum = checksum ^ data[i];
        }
        return checksum;
    }


    void MessageFactory::fill_message_data(uint8_t body[], MessageType message_type, uint8_t* message_out) {
        uint8_t body_size = message_type.get_body_size();
        uint8_t checksum = make_checksum(body, body_size);
        uint8_t message_size = message_type.get_message_size();
        message_out[0] = Parser::START_FLAG;
        message_out[message_size - 2] = checksum;
        message_out[message_size - 1] = Parser::END_FLAG;
        for(uint8_t i = 0 ; i < body_size; i++) {
            message_out[i + 1] = body[i];
        }
    }

    void MessageFactory::get_message_data(MessageType message_type, uint8_t* data, uint8_t* message_out) {
        uint8_t body_size = message_type.get_body_size();
        uint8_t body[body_size];
        body[0] = message_type.get_label();
        for(uint8_t i = 0; i < message_type.get_data_lenght();i++){
            body[i + 1] = data[i];
        }
        fill_message_data(body, message_type, message_out);
    }

    Parser::Parser():state(ParsingState::FINDING_START_FLAG) {

    }

    ParsingError Parser::validate_checksum(uint8_t data) {
        ParsingError parsing_error = ParsingError::NO_ERROR;
        uint8_t parsed_checksum = data;
        uint8_t computed_checksum = MessageFactory::make_checksum(parsed_data, data_index);
        if(parsed_checksum == computed_checksum) {
            state = ParsingState::FINDING_END_FLAG;
        } else {
            parsing_error = ParsingError::CHECKSUM_VALIDATION_ERROR;
            state = ParsingState::FINDING_START_FLAG;
        }
        return parsing_error;
    }

    ParsingResult Parser::parse_byte(uint8_t data) {
        ParsingError parsing_error = ParsingError::NO_ERROR;
        switch(state) {
            case ParsingState::FINDING_START_FLAG:
                if(data == START_FLAG) {
                    state = ParsingState::FINDING_MESSAGE_LABEL;
                } else {
                    parsing_error = ParsingError::START_FLAG_NOT_FOUND;
                }
                break;
            case ParsingState::FINDING_MESSAGE_LABEL:
                for(uint8_t i = 0;message_type.get() == nullptr && i < NUMBER_OF_MESSAGES;++i){
                    if(MESSAGES[i].get_label() == data){
                        message_type = &MESSAGES[i];
                    }
                }
                if(message_type.get() == nullptr) {
                    parsing_error = ParsingError::MESSAGE_LABEL_NOT_FOUND;
                    state = ParsingState::FINDING_START_FLAG;
                } else {
                    state = ParsingState::PARSING_MESSAGE;
                    data_index = 0;
                    parsed_data[data_index++] = data;
                    previous_data_was_escaped = false;
                }
                break;
            case ParsingState::PARSING_MESSAGE:
                if(data_index >= message_type->get_data_lenght()){
                    parsing_error = validate_checksum(data);
                } else {
                    if(previous_data_was_escaped){
                        parsed_data[data_index++] = data;
                        previous_data_was_escaped = false;
                    } else {
                        if(data == ESCAPE_FLAG){
                            previous_data_was_escaped = true;
                        } else {
                            parsed_data[data_index++] = data;
                            previous_data_was_escaped = false;
                        }
                    }
                }
                break;
            case ParsingState::FINDING_END_FLAG:
                state = ParsingState::FINDING_START_FLAG;
                if(data == END_FLAG) {
                    return ParsingResult(state, parsing_error, message_type, parsed_data, true);
                } else {
                    parsing_error = ParsingError::END_FLAG_NOT_FOUND;
                }
                break;
        }
        return ParsingResult(state, parsing_error, message_type, parsed_data, false);
    }

}
