#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_protocol(py::module &);

namespace mcl {

PYBIND11_MODULE(protocol, m) {
    m.doc() = "Protocol library";
    
    init_protocol(m);
}
}
