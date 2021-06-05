
#include <wit901c_rs485/parser.hpp>

namespace wit901c_rs485 {
    class Parser::Implementation {
        public :
            static constexpr float gravity = 9.798;

            const Eigen::Vector3f &parseAccelVec3(const std::string &read_msg) {
                static Eigen::Vector3f vec;
                vec = parseVec3(read_msg);
                vec = vec / 32768 * 16 * gravity;
                return vec;
            }

            const Eigen::Vector3f &parseGyroVec3(const std::string &read_msg) {
                static Eigen::Vector3f vec;
                vec = parseVec3(read_msg);
                vec = vec / 32768 * 2000 * M_PI / 180;
                return vec;
            }

            const Eigen::Vector3f &parseMagnetVec3(const std::string &read_msg) {
                return parseVec3(read_msg);
            }

            const Eigen::Vector3f &parseEulerAngleVec3(const std::string &read_msg) {
                static Eigen::Vector3f vec;
                vec = parseVec3(read_msg);
                vec = vec / 32768 * M_PI;
                return vec;
            }

            const float &parseTemperature(const std::string &read_msg) {
                static float temp;
                temp = parseSingle(read_msg);
                temp = temp / 100;
                return temp;
            }

            const Eigen::Quaternionf &parseQuaternion(const std::string &read_msg) {
                static Eigen::Quaternionf q;
                q.z() = (read_msg.data()[3] << 8) | static_cast<uint8_t>(read_msg.data()[4]);
                q.y() = (read_msg.data()[5] << 8) | static_cast<uint8_t>(read_msg.data()[6]);
                q.x() = (read_msg.data()[7] << 8) | static_cast<uint8_t>(read_msg.data()[8]);
                q.w() = (read_msg.data()[9] << 8) | static_cast<uint8_t>(read_msg.data()[10]);
                q.x() = q.x() / 32768;
                q.y() = -q.y() / 32768;
                q.z() = -q.z() / 32768;
                q.w() = q.w() / 32768;
                return q;
            }

        private :
            const float &parseSingle(const std::string &read_msg) {
                static float ret;
                ret = (read_msg.data()[3] << 8) | static_cast<uint8_t>(read_msg.data()[4]);
                return ret;
            }

            const Eigen::Vector3f &parseVec3(const std::string &read_msg) {
                static Eigen::Vector3f vec;
                vec.x() = (read_msg.data()[3] << 8) | static_cast<uint8_t>(read_msg.data()[4]);
                vec.y() = (read_msg.data()[5] << 8) | static_cast<uint8_t>(read_msg.data()[6]);
                vec.z() = (read_msg.data()[7] << 8) | static_cast<uint8_t>(read_msg.data()[8]);
                return vec;
            }
    };

    Parser::Parser() {
        pimpl = std::make_unique<Implementation>();
    }

    Parser::~Parser() {
    }

    const Eigen::Vector3f &Parser::parseAccel(const std::string &msg) {
        return pimpl->parseAccelVec3(msg);
    }

    const Eigen::Vector3f &Parser::parseGyro(const std::string &msg) {
        return pimpl->parseGyroVec3(msg);
    }

    const Eigen::Vector3f &Parser::parseMagnet(const std::string &msg) {
        return pimpl->parseMagnetVec3(msg);
    }

    const Eigen::Vector3f &Parser::parseEulerAngle(const std::string &msg) {
        return pimpl->parseEulerAngleVec3(msg);
    }

    const float &Parser::parseTemperature(const std::string &msg) {
        return pimpl->parseTemperature(msg);
    }

    const Eigen::Quaternionf &Parser::parseQuaternion(const std::string &msg) {
        return pimpl->parseQuaternion(msg);
    }
}

