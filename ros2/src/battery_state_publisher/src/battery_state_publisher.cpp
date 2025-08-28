#include <memory>
#include <chrono>
#include <vector>
#include <cmath>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "pzem_sensor.hpp"

class BatteryMonitor : public rclcpp::Node {
public:
  static constexpr double TIMEOUT_DURATION = 5.0;
  static constexpr double PUBLISH_RATE = 1.0;
  
  BatteryMonitor()
  : Node("battery_state_publisher"),
    m_pzem_sensor(nullptr),
    m_last_bms_update(this->get_clock()->now())
  {
      declare_parameters();
      
      m_battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("pzem/battery_state", 10);
      m_battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/do150_0007/platform/bms/state", 
        rclcpp::SensorDataQoS().best_effort().durability_volatile(),
        std::bind(&BatteryMonitor::bms_state_callback, this, std::placeholders::_1)
      );
      
      auto timer_period = std::chrono::duration<double>(1.0 / PUBLISH_RATE);
      m_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&BatteryMonitor::timer_callback, this)
      );
      
    try {
      // Initialize sensor
      initialize_sensor();
      RCLCPP_INFO(this->get_logger(), "BatteryMonitor Node started successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BatteryMonitor: %s", e.what());
      throw;
    }
  }

private:
  void declare_parameters()
  {
    this->declare_parameter("frame_id", "battery_frame");
  }
  
  void initialize_sensor() {
    try {
      m_pzem_sensor = std::make_unique<PzemSensor>();
      if (!m_pzem_sensor->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize PZEM sensor");
        throw std::runtime_error("PZEM sensor initialization failed");
      } else {
        RCLCPP_INFO(this->get_logger(), "PZEM sensor initialized successfully");
      }
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Exception during sensor initialization: %s", e.what());
      throw;
    }
  }
  
  BatteryReadings read_sensor_data() {
    BatteryReadings readings = {0.0f, 0.0f, 0.0f, 0.0f, false};
    
    try {
      if (!m_pzem_sensor || !m_pzem_sensor->is_connected()) {
        RCLCPP_FATAL(this->get_logger(), "PZEM sensor not connected");
        rclcpp::shutdown();
        throw std::runtime_error("PZEM sensor connection lost");
      }
      readings = m_pzem_sensor->read();
      if (!readings.valid) {
        RCLCPP_FATAL(this->get_logger(), "Failed to read valid data from PZEM sensor");
        rclcpp::shutdown();
        throw std::runtime_error("Invalid PZEM sensor readings");
      }
      return readings;
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Exception reading PZEM sensor data: %s", e.what());
      rclcpp::shutdown();
      throw;
    }
  }

  void bms_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "Received empty BMS message");
      return;
    }
    
    try {
      m_last_bms_state = *msg;
      m_last_bms_update = this->get_clock()->now();
      
      RCLCPP_DEBUG(this->get_logger(),
        "BMS Update: Status=%d, Voltage=%.2fV, Current=%.2fA, Charge=%.1f%%",
        msg->power_supply_status, msg->voltage, msg->current, msg->percentage);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in BMS callback: %s", e.what());
    }
  }
  
  bool is_bms_data_fresh() {
    try {
      auto now = this->get_clock()->now();
      auto duration = (now - m_last_bms_update).seconds();
      return duration < TIMEOUT_DURATION;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Exception checking BMS freshness: %s", e.what());
      return false;
    }
  }

  void timer_callback() {
    try {
      auto msg = sensor_msgs::msg::BatteryState();
      
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = this->get_parameter("frame_id").as_string();
      
      if (is_bms_data_fresh() && m_last_bms_state) {
        msg = *m_last_bms_state;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = this->get_parameter("frame_id").as_string();
        
        BatteryReadings local_readings = read_sensor_data();
        msg.voltage = local_readings.voltage;
        msg.current = local_readings.current;
        
        RCLCPP_DEBUG(this->get_logger(), 
          "Using PZEM readings: V=%.2fV, I=%.3fA, P=%.2fW", 
          local_readings.voltage, local_readings.current, local_readings.power);
        
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "BMS data stale (%.2fs), not publishing battery state", 
          (this->get_clock()->now() - m_last_bms_update).seconds());
        return;
      }
      
      m_battery_pub->publish(msg);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in timer callback: %s", e.what());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_battery_pub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr m_battery_sub;
  rclcpp::TimerBase::SharedPtr m_timer;
  
  std::unique_ptr<PzemSensor> m_pzem_sensor;
  
  std::optional<sensor_msgs::msg::BatteryState> m_last_bms_state;
  rclcpp::Time m_last_bms_update;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<BatteryMonitor>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("battery_monitor"), 
      "Unhandled exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}