#pragma once

#include <stdint.h>

namespace protocol {

    enum class ParsingState {
        FINDING_START_FLAG,
        FINDING_MESSAGE_LABEL,
        PARSING_MESSAGE,
        FINDING_END_FLAG
    };

    static const uint8_t MESSAGE_TYPE_INDEX = 1;
    static const uint8_t MAXIMUM_DATA_PLAYLOAD_BYTES = 20;
    static const uint8_t MESSAGE_OVERHEAD_IN_BYTES = 4; // IT'S 4 BECAUSE WE NEED START + TYPE + CHECKSUM + END
    static const uint8_t MESSAGE_DATA_OFFSET_IN_BYTES = 2; // IT'S 2 BECAUSE WE NEED START + TYPE
    static const uint8_t MAXIMUM_MESSAGE_PLAYLOAD_BYTES = MAXIMUM_DATA_PLAYLOAD_BYTES + MESSAGE_OVERHEAD_IN_BYTES;

    class MessageType {
        private:
            uint8_t label;
            uint8_t data_length;

        public:

            MessageType(const uint8_t _label, const uint8_t data_length_):
                label(_label),
                data_length(data_length_) {
            }

            int8_t get_message_size() {
                return data_length + MESSAGE_OVERHEAD_IN_BYTES;
            }

            uint8_t get_body_size() {
                // the body is data + label and the label occupies just one byte
                return data_length + 1;
            }

            uint8_t get_label() { return label; }
            uint8_t get_data_length() { return data_length; }

            bool operator== (const MessageType &m1){
                return label == m1.label;
            }

            bool operator!= (const MessageType &m1){
                return label != m1.label;
            }

    };


    class MessageFactory {

        public:
            static char make_checksum(char* data, uint8_t lenght);
            static void fill_message_data(char body[], MessageType message_type, char* message_out);
            static void write_message_data(MessageType message_type, const char* data, char* message_out);
    };


    static MessageType HOME_MESSAGE_TYPE = MessageType(0x01, 0);
    static MessageType RESPONSE_MESSAGE_TYPE = MessageType(0x02, 1);
    static MessageType CONFIGURE_MESSAGE_TYPE = MessageType(0x03, 3);
    static MessageType UNDEFINED_MESSAGE_TYPE = MessageType(0xCC, 0);
    static const uint8_t NUMBER_OF_MESSAGES = 3;
    static MessageType MESSAGES[NUMBER_OF_MESSAGES] = { HOME_MESSAGE_TYPE, RESPONSE_MESSAGE_TYPE, CONFIGURE_MESSAGE_TYPE};

    class Message {
        private:
            MessageType message_type;

        public:
            char message[MAXIMUM_MESSAGE_PLAYLOAD_BYTES] = {0};
            Message(MessageType message_type_, const char* data_);
            Message(MessageType message_type_);
            uint8_t get_message_size();
            char get_byte_at(uint8_t byte_index);
            const char* get_bytes();
            MessageType get_message_type() {
                return message_type;
            }
            static Message make_home_message(){
                const char data[0] = {};
                return Message(HOME_MESSAGE_TYPE, data);
            }
            static Message make_configure_message(bool dir_high_is_clockwise, uint8_t dir_pin, uint8_t step_pin){
                const char data[3] = {(char)dir_high_is_clockwise, (char)dir_pin, (char)step_pin};
                return Message(CONFIGURE_MESSAGE_TYPE, data);
            }
            // TODO: add extra parameter with data to return (e.g. return the current position)
            static Message make_response_message(Message request_message){
                const char data[1] = {request_message.message[MESSAGE_TYPE_INDEX]};
                return Message(RESPONSE_MESSAGE_TYPE, data);
            }
    };


    enum class ParsingError {
        NO_ERROR,
        START_FLAG_NOT_FOUND,
        MESSAGE_LABEL_NOT_FOUND,
        CHECKSUM_VALIDATION_ERROR,
        END_FLAG_NOT_FOUND
    };

    class ParsingResult {
        private:
            ParsingState state = ParsingState::FINDING_START_FLAG;
            ParsingError parsing_error = ParsingError::NO_ERROR;
            Message message = Message(UNDEFINED_MESSAGE_TYPE);
            bool is_parsed = false;

        public:

            ParsingResult() {
            }

            ParsingResult(ParsingState _state, ParsingError _parsing_error, Message message_,
                bool _is_parsed):
                state(_state), parsing_error(_parsing_error),is_parsed(_is_parsed),message(message_) {
            }

            ParsingState &get_state() { return state; }
            ParsingError &get_parsing_error() { return parsing_error; }
            Message &get_message() { return message; }
            bool &get_is_parsed() { return is_parsed; }

    };

    class Parser {
        private:
            MessageType message_type = UNDEFINED_MESSAGE_TYPE;
            char parsed_data[MAXIMUM_DATA_PLAYLOAD_BYTES];
            uint8_t data_index;
            bool previous_data_was_escaped = false;
            ParsingState state = ParsingState::FINDING_START_FLAG;

        public:
            static const char START_FLAG = 0xAA;
            static const char ESCAPE_FLAG = 0xBB;
            static const char END_FLAG = 0xFF;
            static const char constexpr FLAGS[] = { START_FLAG, END_FLAG, ESCAPE_FLAG };

            Parser():state(ParsingState::FINDING_START_FLAG) {
            }
            ParsingResult parse_byte(char data);
            ParsingError validate_checksum(uint8_t data);
            MessageType get_message_type() { return message_type; }
            ParsingState get_state() { return state; }
    };


}

