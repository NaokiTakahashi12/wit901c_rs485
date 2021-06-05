
#include <wit901c_rs485/serial_port.hpp>

#include <thread>
#include <chrono>

#include <Eigen/Dense>

#include <boost/system/error_code.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

namespace wit901c_rs485 {
    class SerialPort::Implementation {
        public :
            Implementation(const SerialPortOptions &options) {
                if(!options.port_name.empty()) {
                    port_name = options.port_name;
                }
                else {
                    port_name = "/dev/ttyUSB0";
                }
                if(options.baud_rate != 0) {
                    baud_rate = options.baud_rate;
                }
                else {
                    baud_rate = 115200;
                }
            };

            ~Implementation() {
                if(!serial_port && serial_port->is_open()) {
                    serial_port->close();
                }
            }

            Implementation(boost::asio::io_service &io_service, const SerialPortOptions &options)
                : Implementation(options) {
                serial_port = std::make_unique<boost::asio::serial_port>(io_service);
            }

            bool openSerialPort(const std::string &overwrite_port_name, const unsigned int &overwrite_baud_rate) {
                if(!serial_port) {
                    throw std::runtime_error("serial_port is nullptr");
                }
                boost::system::error_code ec;
                serial_port->open(overwrite_port_name, ec);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                if(ec) {
                    return true;
                }
                serial_port->set_option(boost::asio::serial_port_base::baud_rate(overwrite_baud_rate));

                return false;
            }

            void writeSerial(const std::string &wrt_msg) {
                if(!serial_port) {
                    throw std::runtime_error("serial_port is nullptr");
                }
                if(!serial_port->is_open()) {
                    throw std::runtime_error("Serial port is not open");
                }
                boost::asio::write(
                    *serial_port,
                    boost::asio::buffer(
                        wrt_msg.data(),
                        wrt_msg.size()
                    )
                );
            }

            const std::string &readSerial(const unsigned int &packet_length) {
                static std::string read_msg;

                if(!serial_port) {
                    throw std::runtime_error("serial_port is nullptr");
                }
                if(!serial_port->is_open()) {
                    throw std::runtime_error("Serial port is not open");
                }

                uint8_t byte = 0;
                unsigned int counter = 0;

                read_msg.clear();

                while(true) {
                    boost::asio::read(*serial_port, boost::asio::buffer(&byte, 1));
                    read_msg += static_cast<char>(byte);
                    counter ++;
                    byte = 0;

                    if(counter >= packet_length) {
                        break;
                    }
                }
                return read_msg;
            }

            std::string port_name;
            unsigned int baud_rate;
            static constexpr float gravity = 9.798;

        private :
            std::unique_ptr<boost::asio::serial_port> serial_port;
    };

    SerialPort::SerialPort(const SerialPortOptions &options) {
        pimpl = std::make_unique<Implementation>(options);
    }

    SerialPort::SerialPort(boost::asio::io_service &io_service, const SerialPortOptions &options) {
        pimpl = std::make_unique<Implementation>(io_service, options);
    }

    SerialPort::~SerialPort() {
    }

    bool SerialPort::open() {
        return pimpl->openSerialPort(pimpl->port_name, pimpl->baud_rate);
    }

    bool SerialPort::open(const std::string &port_name, const unsigned int &baud_rate) {
        return pimpl->openSerialPort(port_name, baud_rate);
    }

    void SerialPort::write(const std::string &msg) {
        pimpl->writeSerial(msg);
    }

    std::string SerialPort::read(const unsigned int &packet_length) {
        return pimpl->readSerial(packet_length);
    }
}

