/**
 * @file csv_collector.cpp
 * @brief Collects diagnostic data and exports it to a CSV file.
 *
 * This program subscribes to a ROS2 diagnostics topic and collects relevant
 * measurements, which are then saved to a CSV file. The CSV header is generated
 * based on the topics provided via the command line.
 *
 * Example usage:
 * ros2 run csv_collector csv_collector -- "Total current (A)" "Battery Voltage (V)"
 *
 * Notes:
 * - The first column is reserved for the timestamp ("Timestamp").
 * - The subsequent columns correspond to the measurement values specified.
 *
 * The resulting CSV file is prefixed defined in FILENAME_PREFIX followed by a formatted
 * timestamp in the filename. The CSV-File is located on the ROS package level.
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
#include "util.hpp"

using StringVec = std::vector<std::string>;
using TopicMap = std::unordered_map<std::string, int>;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

const std::string TIMESTAMP_KEY = "Timestamp";
const std::string FILENAME_PREFIX = "dingo_stats_";
const std::string TOPIC = "/rocksteady/fleet_api/v1_3/diagnostics/diagnostics_aggregated";


class DataCollector: public rclcpp::Node {
    public:
        DataCollector(const std::string name, StringVec topics) 
        : Node("data_collector")
        , m_filename(name)
        , m_toi(initMap(topics))
        {
            writeCsvHeader();

            RCLCPP_INFO(this->get_logger(), "%s created", this->get_name());

            m_subscription = this->create_subscription<DiagnosticArray>(
                TOPIC,
                clearpath_api::QosProfile::streaming(),
                [this](const DiagnosticArray::SharedPtr msg) {

                    StringVec data(m_toi.size());
                    auto timestamp = msg->header.stamp;
                    std::string time = std::to_string(timestamp.sec);
                    const int index = m_toi["Timestamp"];
                    data[index] = time;

                    for (const auto &status : msg->status) {
                        for (const auto &kv : status.values) {
                            if (m_toi.count(kv.key) != 0) {
                                const int index = m_toi[kv.key];
                                data[index] = kv.value;
                            }
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "new data @ %s", time.c_str());
                    writeCsvLine(data);
                }
            );
            RCLCPP_INFO(this->get_logger(), "%s subscribed", TOPIC.c_str());
        }


    private:
        rclcpp::Subscription<DiagnosticArray>::SharedPtr m_subscription;
        std::string m_filename;
        std::ofstream m_outputFile;
        TopicMap m_toi;

        TopicMap initMap(const StringVec topics) {
            TopicMap m_toi;
            int index = 0;
            m_toi[TIMESTAMP_KEY] = index;

            for (const std::string &topic: topics) {
                m_toi[topic] = ++index;
            }
            return m_toi;
        }

        void writeCsvHeader() {
            StringVec header(m_toi.size());
            for (const auto &pair : m_toi) {
                header[pair.second] = pair.first;
            }
            writeCsvLine(header);
        }

        void writeCsvLine(const std::vector<std::string> msgs) {
            // open & close file could also be done by creating and destroying the node (depends on the frequence)
            m_outputFile.open(m_filename, std::ios::out | std::ios::app);
            std::string line = std::accumulate(
                msgs.begin() + 1,
                msgs.end(),
                msgs[0],
                [](std::string a, std::string b) { return a + "," + b; });
            m_outputFile << line << "\n";
            m_outputFile.close();
        }
};


int main(int argc, char *argv[]) {
    std::string filename = FILENAME_PREFIX + getFormattedDateTime();

    // store command line arguments as topics
    StringVec topics;
    for (int i = 1; i < argc; ++i) {
        topics.push_back(argv[i]);
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataCollector>(filename, topics));
    rclcpp::shutdown();
    return 0;
}