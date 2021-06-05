
#pragma once

#include "serial_port_options.hpp"

namespace wit901c_rs485 {
    struct ImuOptions : SerialPortOptions {
        uint8_t device_id;
    };
}

