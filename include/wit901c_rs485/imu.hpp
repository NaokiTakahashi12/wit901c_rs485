
#pragma once

#include <memory>

#include <Eigen/Dense>

#include <boost/asio/io_service.hpp>

#include "serial_port.hpp"
#include "parser.hpp"
#include "send_messages.hpp"
#include "imu_options.hpp"

namespace wit901c_rs485 {
    class Imu {
        public :
            Imu(boost::asio::io_service &, const ImuOptions &);
            ~Imu();

            Eigen::Vector3f getAccel();
            Eigen::Vector3f getGyro();
            Eigen::Vector3f getMagnet();
            Eigen::Vector3f getEulerAngle();
            float getTemperature();
            Eigen::Quaternionf getQuaternion();

        private :
            std::unique_ptr<SerialPort> serial_port;
            std::unique_ptr<Parser> parser;
            std::unique_ptr<SendMessages> send_messages;
    };
}

