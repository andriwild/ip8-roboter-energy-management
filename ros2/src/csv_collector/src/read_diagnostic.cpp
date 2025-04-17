/**
 * @file diagnostic_key_value_debug.cpp
 * @brief Lists diagnostic key-value pairs for debugging purposes.
 *
 * This file subscribes to a diagnostic topic (e.g., "/diagnostics/diagnostics_aggregated") 
 * and iterates over the * DiagnosticArray message to output the key-value pairs contained 
 * in each DiagnosticStatus. 
 * It is intended to help debug and verify the diagnostic data from various components.
 *
 * Key points:
 * - Subscribes to diagnostic_msgs/msg/DiagnosticArray.
 * - Iterates through each DiagnosticStatus and prints the key-value pairs.
 * 
 * Ensure that:
 * - your QoS settings are compatible between the publisher and subscriber.
 * - ROS_DOMAIN_ID is set. For the FLEET_API's diagnostic topics use 100.
 *
 * @author Andri Wild
 * @date 10. April 2025
 */

#include <memory>
#include <ctime>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "clearpath_api/qos_profiles.hpp"


class KeyValueReader: public rclcpp::Node {
    public:
        KeyValueReader() : Node("key_value_reader")
        {
            m_subscription = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                "/rocksteady/fleet_api/v1_3/diagnostics/diagnostics_aggregated",
                clearpath_api::QosProfile::streaming(),
                [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
                    for (const auto &status : msg->status) {
                        for (const auto &kv : status.values) {
                            RCLCPP_INFO(this->get_logger(), "Key: %s \t %s", kv.key.c_str(), kv.value.c_str());
                        }
                    }
                });
        }

    private:
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_subscription;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyValueReader>());
    rclcpp::shutdown();
    return 0;
}