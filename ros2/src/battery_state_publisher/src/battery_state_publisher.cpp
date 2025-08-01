#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;
typedef sensor_msgs::msg::BatteryState BatteryState;

class BatteryStatePublisher : public rclcpp::Node {
    public:
        BatteryStatePublisher() : Node("battery_state_publisher"), isCharging(false) {

            publisher_ = this->create_publisher<BatteryState>("battery_state", 10);
            subscriber = this->create_subscription<BatteryState>(
                "bms/state", 
                10,
                std::bind(&BatteryStatePublisher::bms_state_callback, this, std::placeholders::_1)
            );
            timer_ = this->create_wall_timer(
                500ms, std::bind(&BatteryStatePublisher::timer_callback, this)
            );
        }

    private:
        void timer_callback() {
          auto message = sensor_msgs::msg::BatteryState();
          message.capacity = 44.0;
          message.voltage = 12.4;
          message.current = 3.4;
          publisher_->publish(message);
        }

        void bms_state_callback(const BatteryState::SharedPtr msg){
            if (msg->power_supply_status == BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
              isCharging = true;
            } else {
              isCharging = false;
            }
        }

        bool isCharging;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<BatteryState>::SharedPtr publisher_;
        rclcpp::Subscription<BatteryState>::SharedPtr subscriber;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
