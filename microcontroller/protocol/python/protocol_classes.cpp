#include "../cpp/protocol.hpp"

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>


namespace py = pybind11;

void init_protocol(py::module &m) {

     m.attr("myConstant") = py::int_(123);

     py::class_<protocol::MessageType>(m, "MessageType")
          .def(py::init<const uint8_t, const uint8_t>())
          .def("get_body_size", &protocol::MessageType::get_body_size)
          .def("get_message_size", &protocol::MessageType::get_message_size);

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

     py::class_<protocol::ptr_wrapper<protocol::MessageType>>(m, "MessageTypePtr")
          .def("get", &protocol::ptr_wrapper<protocol::MessageType>::get);

     py::class_<protocol::ParsingResult>(m, "ParsingResult")
          .def(py::init<protocol::ParsingState, protocol::ParsingError,
               protocol::ptr_wrapper<protocol::MessageType>,  protocol::ptr_wrapper<uint8_t>, bool>())
          .def("get_state", &protocol::ParsingResult::get_state)
          .def("get_parsing_error", &protocol::ParsingResult::get_parsing_error);

     py::class_<protocol::Message>(m, "Message")
          .def(py::init<protocol::MessageType, const char*>())
          .def("get_message_size", &protocol::Message::get_message_size)
          .def("get_byte_at", &protocol::Message::get_byte_at)
          .def("get_bytes", [](const protocol::Message message){
               std::string message_as_string(message.message);
               return py::bytes(message_as_string);
          })
          .def_static("make_home_message", &protocol::Message::make_home_message);

}