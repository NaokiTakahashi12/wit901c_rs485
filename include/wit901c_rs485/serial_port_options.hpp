
#pragma once

#include <string>

namespace wit901c_rs485 {
    struct SerialPortOptions {
        std::string port_name;
        unsigned int baud_rate;
    };
}

