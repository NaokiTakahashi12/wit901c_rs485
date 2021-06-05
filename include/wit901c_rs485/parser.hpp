
#pragma once

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "serial_port.hpp"

namespace wit901c_rs485 {
    class Parser {
        public :
            Parser();
            ~Parser();

            const Eigen::Vector3f &parseAccel(const std::string &);
            const Eigen::Vector3f &parseGyro(const std::string &);
            const Eigen::Vector3f &parseMagnet(const std::string &);
            const Eigen::Vector3f &parseEulerAngle(const std::string &);
            const float &parseTemperature(const std::string &);
            const Eigen::Quaternionf &parseQuaternion(const std::string &);

        private :
            class Implementation;
            std::unique_ptr<Implementation> pimpl;
    };
}

