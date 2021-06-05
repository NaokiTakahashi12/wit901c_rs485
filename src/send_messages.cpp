
#include <wit901c_rs485/send_messages.hpp>

namespace wit901c_rs485 {
    SendMessages::SendMessages(const ImuOptions &options) : device_id(options.device_id) {
    }

    uint16_t crcModbus(const std::string &msg) {
        uint16_t crc = 0xffff;
        for(unsigned long i = 0; i < msg.size(); i ++) {
            crc ^= static_cast<uint8_t>(msg.data()[i]);

            for(auto j = 0; j < 8; j ++) {
                if(crc & 1) {
                    crc = (crc >> 1) ^ 0xa001;
                }
                else {
                    crc = (crc >> 1);
                }
            }
        }
        return crc;
    }

    const std::string &SendMessages::requestAccelMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x34);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x03);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }

    const std::string &SendMessages::requestGyroMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x37);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x03);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }

    const std::string &SendMessages::requestMagnetMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x3a);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x03);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }

    const std::string &SendMessages::requestEulerAngleMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x3d);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x03);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }

    const std::string &SendMessages::requestTemperatureMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x40);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x01);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }

    const std::string &SendMessages::requestQuaternionMsg() {
        static std::string ret_msg;
        if(ret_msg.empty()) {
            ret_msg += static_cast<uint8_t>(device_id);
            ret_msg += static_cast<uint8_t>(0x03);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x51);
            ret_msg += static_cast<uint8_t>(0x00);
            ret_msg += static_cast<uint8_t>(0x04);

            const auto crc = crcModbus(ret_msg);
            ret_msg += static_cast<uint8_t>(crc);
            ret_msg += static_cast<uint8_t>(crc >> 8);
        }
        return ret_msg;
    }
}

