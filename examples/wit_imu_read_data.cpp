
#include <exception>
#include <iostream>
#include <string>
#include <sstream>

#include <wit901c_rs485/imu_options.hpp>
#include <wit901c_rs485/imu.hpp>

std::string console_helper(int argc, char **argv);

auto main(int argc, char **argv) -> int {
    try {
        if(argc < 3) {
            std::cout << console_helper(argc, argv) << std::endl;
            throw std::runtime_error("Please set arguments");
        }
        // Reference from https://github.com/WITMOTION/WT901C-RS485
        std::string port_name = "/dev/ttyUSB0";
        unsigned int baud_rate = 9600;
        uint8_t device_id = 80;
        unsigned int max_try = 10;

        if(argc >= 2)
            port_name = argv[1];
        if(argc >= 3)
            baud_rate = std::stoi(argv[2]);
        if(argc >= 4)
            device_id = std::stoi(argv[3]);
        if(argc >= 5)
            max_try = std::stoi(argv[4]);

        std::cout << "Port name: " << port_name << std::endl;
        std::cout << "Baud rate: " << baud_rate << std::endl;
        std::cout << "Device ID: " << static_cast<int>(device_id) << std::endl;
        std::cout << "  Max try: " << max_try << std::endl;

        boost::asio::io_service io_service{};

        wit901c_rs485::ImuOptions options{};
        options.port_name = port_name;
        options.baud_rate = baud_rate;
        options.device_id = device_id;

        wit901c_rs485::Imu imu{io_service, options};

        for(auto i = 0; i < max_try; i ++) {
            std::cout << "Accel is " << imu.getAccel().transpose() << std::endl;
            std::cout << "Gyro is " << imu.getGyro().transpose() << std::endl;
            std::cout << "Magnet is " << imu.getMagnet().transpose() << std::endl;
            std::cout << "EulerAngle is " << imu.getEulerAngle().transpose() << std::endl;
            std::cout << "Temperature is " << imu.getTemperature() << std::endl;

            Eigen::Quaternionf q = imu.getQuaternion();
            std::cout
                << "Quaternion is "
                << q.x() << ", "
                << q.y() << ", "
                << q.z() << ", "
                << q.w()
                << std::endl;
        }
    }
    catch(const std::exception &e) {
        std::cerr << "std::exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}

std::string console_helper(int argc, char **argv) {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << argv[0] << " <port_name> <baud_rate> <device_id> <max_try>" << std::endl;
    return ss.str();
}

