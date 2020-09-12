#pragma once

#include <stdint.h>
//#include <iostream>

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
            static char make_checksum(MessageType message_type, char* data, uint8_t begin, uint8_t end);
            //static char make_checksum(char* data, uint8_t begin, uint8_t end);
            static uint8_t write_message_data(MessageType message_type, const char* data, char* message_out);
    };


    static MessageType HOME_MESSAGE_TYPE = MessageType(0x01, 0);
    static MessageType HOME_RESPONSE_MESSAGE_TYPE = MessageType(0x02, 0);
    static MessageType CONFIGURE_MESSAGE_TYPE = MessageType(0x03, 3);
    static MessageType CONFIGURE_RESPONSE_MESSAGE_TYPE = MessageType(0x04, 0);
    static MessageType HOMING_STATE_MESSAGE_TYPE = MessageType(0x05, 0);
    static MessageType HOMING_STATE_RESPONSE_MESSAGE_TYPE = MessageType(0x06, 1);
    static MessageType GET_STEPS_MESSAGE_TYPE = MessageType(0x07, 0);
    static MessageType GET_STEPS_RESPONSE_MESSAGE_TYPE = MessageType(0x08, 2);
    static MessageType UNDEFINED_MESSAGE_TYPE = MessageType(0xCC, 0);
    static const uint8_t NUMBER_OF_MESSAGES = 8;
    static MessageType MESSAGES[NUMBER_OF_MESSAGES] = {
        HOME_MESSAGE_TYPE, HOME_RESPONSE_MESSAGE_TYPE,
        CONFIGURE_MESSAGE_TYPE, CONFIGURE_RESPONSE_MESSAGE_TYPE,
        HOMING_STATE_MESSAGE_TYPE, HOMING_STATE_RESPONSE_MESSAGE_TYPE,
        GET_STEPS_MESSAGE_TYPE, GET_STEPS_RESPONSE_MESSAGE_TYPE};

    class Message {
        private:
            MessageType message_type;
            uint8_t escaped_bytes;

        public:
            char message[MAXIMUM_MESSAGE_PLAYLOAD_BYTES] = {0};
            char data[MAXIMUM_DATA_PLAYLOAD_BYTES] = {0};
            Message(MessageType message_type_, const char* data_);
            Message(MessageType message_type_);

            char* get_data(){
                return data;
            }

            uint8_t get_message_length() {
                return escaped_bytes + message_type.get_data_length() + MESSAGE_OVERHEAD_IN_BYTES;
            }

            char get_byte_at(uint8_t byte_index);
            const char* get_bytes();
            MessageType get_message_type() {
                return message_type;
            }

            static Message make_configure_message(bool dir_high_is_clockwise, uint8_t dir_pin, uint8_t step_pin){
                const char data[3] = {(char)dir_high_is_clockwise, (char)dir_pin, (char)step_pin};
                return Message(CONFIGURE_MESSAGE_TYPE, data);
            }
            static Message make_configure_response_message(){
                const char data[0] = {};
                return Message(CONFIGURE_RESPONSE_MESSAGE_TYPE, data);
            }

            static Message make_homing_message(){
                const char data[0] = {};
                return Message(HOME_MESSAGE_TYPE, data);
            }
            static Message make_homing_response_message(){
                const char data[0] = {};
                return Message(HOME_RESPONSE_MESSAGE_TYPE, data);
            }

            static Message make_homing_state_message(){
                const char data[0] = {};
                return Message(HOMING_STATE_MESSAGE_TYPE, data);
            }

            static Message make_homing_state_response_message(uint8_t enum_idx) {
                const char data[1] = { (char) enum_idx };
                return Message(HOMING_STATE_RESPONSE_MESSAGE_TYPE, data);
            }

            static Message make_get_steps_message() {
                const char data[0] = { };
                return Message(GET_STEPS_MESSAGE_TYPE, data);
            }

            static Message make_get_steps_response_message(int16_t steps) {
                const char data[2] = { (char) (steps >> 8) & 0x00FF, (char) steps & 0x00FF };
                return Message(GET_STEPS_RESPONSE_MESSAGE_TYPE, data);
            }

            static int16_t make_int16_from_two_bytes(char byte_1, char byte_2) {
                const int16_t data  = ((byte_1 << 8) & 0xFF00) + (byte_2 & 0x00FF);
                return data;
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
            char checksum;
            ParsingState state = ParsingState::FINDING_START_FLAG;

        public:
            static const char START_FLAG = 0xAA;
            static const char ESCAPE_FLAG = 0xBB;
            static const char END_FLAG = 0xFF;
            static const char NUM_FLAGS = 3;
            static const char constexpr FLAGS[NUM_FLAGS] = { START_FLAG, END_FLAG, ESCAPE_FLAG };

            Parser():state(ParsingState::FINDING_START_FLAG) {
            }
            ParsingResult parse_byte(char data);
            ParsingError validate_checksum(uint8_t computed_checksum, uint8_t parsed_checksum);
            //ParsingError validate_checksum(uint8_t data);
            MessageType get_message_type() { return message_type; }
            ParsingState get_state() { return state; }
            static bool is_flag(char byte){
                bool flag = false;
                for(int i = 0; (!flag) && i < NUM_FLAGS;i++){
                    flag = flag || (byte == FLAGS[i]);
                }
                return flag;
            }


    };


}

