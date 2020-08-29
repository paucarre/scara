#pragma once

#include <stdint.h>
#include <iostream>

namespace protocol {

    template <class T> class ptr_wrapper
    {
        public:
            ptr_wrapper() : ptr(nullptr) {}
            ptr_wrapper(T* ptr) : ptr(ptr) {}
            ptr_wrapper(const ptr_wrapper& other) : ptr(other.ptr) {}
            T& operator* () const { return *ptr; }
            T* operator->() const { return  ptr; }
            T* get() const { return ptr; }
            void destroy() { delete ptr; }
            T& operator[](std::size_t idx) const { return ptr[idx]; }
        private:
            T* ptr;
    };


    enum class ParsingState {
        FINDING_START_FLAG,
        FINDING_MESSAGE_LABEL,
        PARSING_MESSAGE,
        FINDING_END_FLAG
    };

    class MessageType {
        private:
            const uint8_t label;
            const uint8_t data_lenght;

        public:

            MessageType(const uint8_t _label, const uint8_t _lenght):
                label(_label),
                data_lenght(_lenght) {
            }

            const uint8_t get_body_size();
            const uint8_t get_message_size();
            const uint8_t get_label() { return label; }
            const uint8_t get_data_lenght() { return data_lenght; }

    };


    static const uint8_t MAXIMUM_DATA_PLAYLOAD_BYTES = 20;
    static const uint8_t MESSAGE_OVERHEAD_IN_BYTES = 4; // IT'S 4 BECAUSE WE NEED START + TYPE + CHECKSUM + END
    static const uint8_t MAXIMUM_MESSAGE_PLAYLOAD_BYTES = MAXIMUM_DATA_PLAYLOAD_BYTES + MESSAGE_OVERHEAD_IN_BYTES;
    static const MessageType HOME_MESSAGE_TYPE = MessageType(0x01, 0);
    static const MessageType HOME_RETURN_MESSAGE_TYPE = MessageType(0x02, 0);
    static const uint8_t NUMBER_OF_MESSAGES = 2;
    static MessageType MESSAGES[NUMBER_OF_MESSAGES] = { HOME_MESSAGE_TYPE, HOME_RETURN_MESSAGE_TYPE};

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
            ptr_wrapper<MessageType> message_type = nullptr;
            ptr_wrapper<uint8_t> data = nullptr;
            bool is_parsed = false;

        public:
            ParsingResult() {

            }

            ParsingResult(ParsingState _state, ParsingError _parsing_error,
                ptr_wrapper<MessageType> _message_type, ptr_wrapper<uint8_t> _data, bool _is_parsed):
                state(_state), parsing_error(_parsing_error), message_type(_message_type), data(_data), is_parsed(_is_parsed) {
            }

            ParsingState &get_state() { return state; }
            ParsingError &get_parsing_error() { return parsing_error; }
            ptr_wrapper<MessageType> &get_message_type() { return message_type; }
            ptr_wrapper<uint8_t> &get_data() { return data; }
            bool &get_is_parsed() { return is_parsed; }

    };

    class Parser {
        private:
            ptr_wrapper<MessageType> message_type = nullptr;
            uint8_t parsed_data[MAXIMUM_DATA_PLAYLOAD_BYTES];
            uint8_t data_index;
            bool previous_data_was_escaped = false;
            ParsingState state = ParsingState::FINDING_START_FLAG;

        public:
            static const char START_FLAG = 0xAA;
            static const char ESCAPE_FLAG = 0x00;
            static const char END_FLAG = 0xFF;
            static const char constexpr FLAGS[] = { START_FLAG, END_FLAG, ESCAPE_FLAG };

            Parser();
            ParsingResult parse_byte(uint8_t data);
            ParsingError validate_checksum(uint8_t data);
            ptr_wrapper<MessageType> &get_message_type() { return message_type; }
            ParsingState get_state() { return state; }
    };

    class MessageFactory {

        public:
            static char make_checksum(uint8_t* data, uint8_t lenght);
            static void fill_message_data(uint8_t body[], MessageType message_type, char* message_out);
            static void write_message_data(MessageType message_type, const char* data, char* message_out);
    };

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
            static Message make_home_message(){
                const char data[0] = {};
                return Message(HOME_MESSAGE_TYPE, data);
            }
    };


}

