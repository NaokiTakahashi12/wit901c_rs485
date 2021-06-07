
#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <sstream>

#include <boost/asio/io_service.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <wit901c_rs485/imu_options.hpp>
#include <wit901c_rs485/imu.hpp>

namespace wit901c_rs485 {
    class ImuNode : public rclcpp::Node {
        public :
            ImuNode() : Node("wit_imu_node") {
                bool use_imu,
                     use_mag,
                     use_temperature;
                std::string imu_topic,
                            mag_topic,
                            temperature_topic;

                this->declare_parameter<std::string>("serial_port_name", "/dev/ttyUSB0");
                this->declare_parameter<int>("serial_baud_rate", 9600);
                this->declare_parameter<int>("serial_device_id", 80);
                this->declare_parameter<std::string>("frame_id", "wit_imu_link");
                this->declare_parameter<double>("publish_frequency", 50);
                this->declare_parameter<double>("publish_time_offset", 0.0);
                this->declare_parameter<bool>("use_imu", true);
                this->declare_parameter<bool>("use_mag", false);
                this->declare_parameter<bool>("use_temperature", false);
                this->declare_parameter<std::string>("imu_topic", "imu/data_raw");
                this->declare_parameter<std::string>("mag_topic", "imu/mag");
                this->declare_parameter<std::string>("temperature_topic", "imu/temperature");

                this->get_parameter("serial_port_name", imu_options.port_name);
                this->get_parameter("serial_baud_rate", imu_options.baud_rate);
                this->get_parameter("serial_device_id", imu_options.device_id);
                this->get_parameter("frame_id", imu_frame_id);
                this->get_parameter("publish_frequency", publish_frequency);
                this->get_parameter("use_imu", use_imu);
                this->get_parameter("use_mag", use_mag);
                this->get_parameter("use_temperature", use_temperature);
                this->get_parameter("imu_topic", imu_topic);
                this->get_parameter("mag_topic", mag_topic);
                this->get_parameter("temperature_topic", temperature_topic);

                if(use_imu)
                    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
                if(use_mag)
                    mag_publisher = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 10);
                if(use_temperature)
                    temp_publisher = this->create_publisher<sensor_msgs::msg::Temperature>(temperature_topic, 10);

                currentParameterPrinter();

                imu = std::make_unique<Imu>(io_service, imu_options);
                RCLCPP_INFO(this->get_logger(), "Success open serial port");

                timer = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(1e3 / publish_frequency)),
                    std::bind(&ImuNode::timerCallback, this)
                );
            }

        private :
            boost::asio::io_service io_service;

            std::string imu_frame_id;
            double publish_frequency;

            ImuOptions imu_options;
            std::unique_ptr<Imu> imu;

            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
            rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher;
            rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher;

            rclcpp::TimerBase::SharedPtr timer;

            void timerCallback() {
                publishImu();
                publishMag();
                publishTemp();
            }

            void publishImu() {
                static Eigen::Quaternionf quaternion;
                static Eigen::Vector3f accel, gyro;

                if(!imu_publisher) {
                    return;
                }

                auto imu_message = sensor_msgs::msg::Imu();

                accel = imu->getAccel();
                gyro = imu->getAccel();
                quaternion = imu->getQuaternion();

                imu_message.orientation.x = quaternion.x();
                imu_message.orientation.y = quaternion.y();
                imu_message.orientation.z = quaternion.z();
                imu_message.orientation.w = quaternion.w();
                imu_message.angular_velocity.x = gyro.x();
                imu_message.angular_velocity.y = gyro.y();
                imu_message.angular_velocity.z = gyro.z();
                imu_message.linear_acceleration.x = accel.x();
                imu_message.linear_acceleration.y = accel.y();
                imu_message.linear_acceleration.z = accel.z();
                imu_message.header.frame_id = imu_frame_id;
                imu_message.header.stamp = this->now();

                imu_publisher->publish(imu_message);
            }

            void publishMag() {
                static Eigen::Vector3f mag;

                if(!mag_publisher) {
                    return;
                }

                auto mag_message = sensor_msgs::msg::MagneticField();

                mag = imu->getMagnet();

                mag_message.magnetic_field.x = mag.x();
                mag_message.magnetic_field.y = mag.y();
                mag_message.magnetic_field.z = mag.z();
                mag_message.header.frame_id = imu_frame_id;
                mag_message.header.stamp = this->now();

                mag_publisher->publish(mag_message);
            }

            void publishTemp() {
                if(!temp_publisher) {
                    return;
                }

                auto temp_message = sensor_msgs::msg::Temperature();

                temp_message.temperature = imu->getTemperature();
                temp_message.header.frame_id = imu_frame_id;
                temp_message.header.stamp = this->now();

                temp_publisher->publish(temp_message);
            }

            void currentParameterPrinter() {
                RCLCPP_INFO(this->get_logger(), "Serial port: %s", imu_options.port_name.c_str());
                RCLCPP_INFO(this->get_logger(), "Serial baud rate: %i", imu_options.baud_rate);
                RCLCPP_INFO(this->get_logger(), "Serial device ID: %i", imu_options.device_id);
                RCLCPP_INFO(this->get_logger(), "IMU frame ID: %s", imu_frame_id.c_str());
                RCLCPP_INFO(this->get_logger(), "Publish frequency: %f", publish_frequency);
            }
    };
}

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wit901c_rs485::ImuNode>());
    rclcpp::shutdown();
    return 0;
}
