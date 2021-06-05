
#pragma once

#include <string>

#include "imu_options.hpp"

namespace wit901c_rs485 {
    class SendMessages {
        public :
            SendMessages(const ImuOptions &);

            const std::string &requestAccelMsg();
            const std::string &requestGyroMsg();
            const std::string &requestMagnetMsg();
            const std::string &requestEulerAngleMsg();
            const std::string &requestTemperatureMsg();
            const std::string &requestQuaternionMsg();

        private :
            const uint8_t device_id;
    };
}

