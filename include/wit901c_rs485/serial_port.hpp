
#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include <boost/asio/io_service.hpp>

#include "serial_port_options.hpp"

namespace wit901c_rs485 {
    class SerialPort {
        public :
            SerialPort(const SerialPortOptions &);
            SerialPort(boost::asio::io_service &, const SerialPortOptions &);
            ~SerialPort();

            bool open();
            bool open(const std::string &port_name, const unsigned int &baud_rate);

            void write(const std::string &msg);
            std::string read(const unsigned int &packet_length);

        private :
            class Implementation;

            std::unique_ptr<Implementation> pimpl;
    };
}

