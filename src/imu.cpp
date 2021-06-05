
#include <wit901c_rs485/imu.hpp>

#include <stdexcept>

namespace wit901c_rs485 {
    Imu::Imu(boost::asio::io_service &io, const ImuOptions &options) {
        serial_port = std::make_unique<SerialPort>(io, options);
        send_messages = std::make_unique<SendMessages>(options);
        parser = std::make_unique<Parser>();

        if(serial_port->open()) {
            throw std::runtime_error("Failed open serial port");
        }
    }

    Imu::~Imu() {
    }

    Eigen::Vector3f Imu::getAccel() {
        constexpr auto packet_length = 11;
        serial_port->write(send_messages->requestAccelMsg());
        return parser->parseAccel(serial_port->read(packet_length));
    }

    Eigen::Vector3f Imu::getGyro() {
        constexpr auto packet_length = 11;
        serial_port->write(send_messages->requestGyroMsg());
        return parser->parseGyro(serial_port->read(packet_length));
    }

    Eigen::Vector3f Imu::getMagnet() {
        constexpr auto packet_length = 11;
        serial_port->write(send_messages->requestMagnetMsg());
        return parser->parseMagnet(serial_port->read(packet_length));
    }

    Eigen::Vector3f Imu::getEulerAngle() {
        constexpr auto packet_length = 11;
        serial_port->write(send_messages->requestEulerAngleMsg());
        return parser->parseEulerAngle(serial_port->read(packet_length));
    }

    float Imu::getTemperature() {
        constexpr auto packet_length = 7;
        serial_port->write(send_messages->requestTemperatureMsg());
        return parser->parseTemperature(serial_port->read(packet_length));
    }

    Eigen::Quaternionf Imu::getQuaternion() {
        constexpr auto packet_length = 13;
        serial_port->write(send_messages->requestQuaternionMsg());
        return parser->parseQuaternion(serial_port->read(packet_length));
    }
}

