#include "../cpp/protocol.hpp"

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <optional>

namespace py = pybind11;

void init_protocol(py::module &m) {

     py::class_<protocol::MessageType>(m, "MessageType")
          .def(py::init<const uint8_t, const uint8_t>())
          .def("get_body_size", &protocol::MessageType::get_body_size)
          .def("get_message_size", &protocol::MessageType::get_message_size)
          .def("__eq__", &protocol::MessageType::operator==, py::is_operator())
          .def("__ne__", &protocol::MessageType::operator!=, py::is_operator())
          .def("get_label", &protocol::MessageType::get_label);

     py::enum_<protocol::ParsingState>(m, "ParsingState", py::arithmetic())
          .value("FINDING_START_FLAG", protocol::ParsingState::FINDING_START_FLAG)
          .value("FINDING_MESSAGE_LABEL", protocol::ParsingState::FINDING_MESSAGE_LABEL)
          .value("PARSING_MESSAGE", protocol::ParsingState::PARSING_MESSAGE)
          .value("FINDING_END_FLAG", protocol::ParsingState::FINDING_END_FLAG);

     py::enum_<protocol::ParsingError>(m, "ParsingError", py::arithmetic())
        .value("NO_ERROR", protocol::ParsingError::NO_ERROR)
        .value("START_FLAG_NOT_FOUND", protocol::ParsingError::START_FLAG_NOT_FOUND)
        .value("MESSAGE_LABEL_NOT_FOUND", protocol::ParsingError::MESSAGE_LABEL_NOT_FOUND)
        .value("CHECKSUM_VALIDATION_ERROR", protocol::ParsingError::CHECKSUM_VALIDATION_ERROR)
        .value("END_FLAG_NOT_FOUND", protocol::ParsingError::END_FLAG_NOT_FOUND);

     py::class_<protocol::ParsingResult>(m, "ParsingResult")
          .def(py::init<protocol::ParsingState, protocol::ParsingError, protocol::Message, bool>())
          .def("get_state", &protocol::ParsingResult::get_state)
          .def("get_message",[](protocol::ParsingResult parsing_result) -> std::optional<protocol::Message> {
               if(parsing_result.get_message().get_message_type().get_label() == protocol::UNDEFINED_MESSAGE_TYPE.get_label()) {
                    return std::nullopt;
               } else {
                    return std::optional(parsing_result.get_message());
               }
          })
          .def("is_parsed", &protocol::ParsingResult::get_is_parsed)
          .def("get_parsing_error", &protocol::ParsingResult::get_parsing_error);

     py::class_<protocol::Message>(m, "Message")
          .def(py::init<protocol::MessageType, const char*>())
          .def("get_message_size", &protocol::Message::get_message_size)
          .def("get_byte_at", &protocol::Message::get_byte_at)
          .def("get_bytes", [](const protocol::Message message){
               std::string message_as_string(message.message);
               return py::bytes(message_as_string);
          })
          .def("get_message_type", &protocol::Message::get_message_type)
          .def_static("make_home_message", &protocol::Message::make_home_message);

     py::class_<protocol::Parser>(m, "Parser")
          .def(py::init<>())
          .def("parse_bytes", [](protocol::Parser parser, py::bytes message_bytes) {
               std::string message_string = message_bytes;
               char* data = message_string.data();
               uint32_t length = message_string.length();
               protocol::ParsingResult parse_result;
               for(uint32_t idx = 0; idx < length; idx++){
                    parse_result = parser.parse_byte(data[idx]);
               }
               return parse_result;
          })
          .def("get_message_type", &protocol::Parser::get_message_type);

     m.attr("HOME_MESSAGE_TYPE") = py::cast(protocol::HOME_MESSAGE_TYPE);

}