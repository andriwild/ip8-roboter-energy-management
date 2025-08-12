#include <memory>
#include <chrono>
#include <vector>
#include <cmath>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "pzem_sensor.hpp"

// Forward declaration for dummy sensor
class BatteryMonitor : public rclcpp::Node
{
public:
  BatteryMonitor()
  : Node("battery_monitor"),
    pzem_sensor_(nullptr),
    last_bms_update_(this->get_clock()->now()),
    timeout_duration_(5.0),  // 5 seconds timeout
    publish_rate_(1.0)       // 1 Hz default publication rate
  {
      // Declare parameters
      declare_parameters();
      
      // Publisher for own BatteryState
      battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", 10);
      
      // Subscription to /bms/state with robust QoS settings
      battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/do150_0007/platform/bms/state", 
        rclcpp::SensorDataQoS()          // = BEST_EFFORT + VOLATILE + keep_last(5)
      .best_effort()               // explizit setzen, damit es beim Lesen klar ist
      .durability_volatile(),
        std::bind(&BatteryMonitor::bms_state_callback, this, std::placeholders::_1)
      );
      
      // Timer for cyclic publishing (configurable rate)
      auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&BatteryMonitor::timer_callback, this)
      );
      
    try {
      // Initialize sensor
      RCLCPP_INFO(this->get_logger(), "Init sensor");
      initialize_sensor();

      RCLCPP_INFO(this->get_logger(), "Init sensor done");
      
      RCLCPP_INFO(this->get_logger(), "BatteryMonitor Node started successfully");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BatteryMonitor: %s", e.what());
      throw;
    }
  }

private:
  void declare_parameters()
  {
    // Timeout parameter
    rcl_interfaces::msg::ParameterDescriptor timeout_desc;
    timeout_desc.description = "Timeout for BMS messages in seconds";
    timeout_desc.floating_point_range.resize(1);
    timeout_desc.floating_point_range[0].from_value = 1.0;
    timeout_desc.floating_point_range[0].to_value = 60.0;
    this->declare_parameter("bms_timeout", timeout_duration_, timeout_desc);
    
    // Publication rate parameter
    rcl_interfaces::msg::ParameterDescriptor rate_desc;
    rate_desc.description = "Publication rate in Hz";
    rate_desc.floating_point_range.resize(1);
    rate_desc.floating_point_range[0].from_value = 0.1;
    rate_desc.floating_point_range[0].to_value = 10.0;
    this->declare_parameter("publish_rate", publish_rate_, rate_desc);
    
    // Frame ID parameter
    this->declare_parameter("frame_id", "battery_frame");
    
    // Get parameter values
    timeout_duration_ = this->get_parameter("bms_timeout").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
  }
  
  void initialize_sensor()
  {

      RCLCPP_INFO(this->get_logger(), "1");
    try {

      RCLCPP_INFO(this->get_logger(), "2");
      pzem_sensor_ = std::make_unique<PzemSensor>();

      RCLCPP_INFO(this->get_logger(), "3");
      if (!pzem_sensor_->initialize()) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize PZEM sensor, continuing without local readings");
        pzem_sensor_.reset();
      } else {
        RCLCPP_INFO(this->get_logger(), "PZEM sensor initialized successfully");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during sensor initialization: %s", e.what());
      pzem_sensor_.reset();
    }
  }
  
  BatteryReadings read_sensor_data()
  {
    BatteryReadings readings = {0.0f, 0.0f, 0.0f, 0.0f, false};
    
    try {
      if (pzem_sensor_ && pzem_sensor_->is_connected()) {
        return pzem_sensor_->read();
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Exception reading PZEM sensor data: %s", e.what());
    }
    
    return readings;
  }

  void bms_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "Received empty BMS message");
      return;
    }
    
    try {
      // Update last BMS state
      last_bms_state_ = *msg;
      last_bms_update_ = this->get_clock()->now();
      
      RCLCPP_DEBUG(this->get_logger(),
        "BMS Update: Status=%d, Voltage=%.2fV, Current=%.2fA, Charge=%.1f%%",
        msg->power_supply_status, msg->voltage, msg->current, msg->percentage);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in BMS callback: %s", e.what());
    }
  }
  
  bool is_bms_data_fresh()
  {
    try {
      auto now = this->get_clock()->now();
      auto duration = (now - last_bms_update_).seconds();
      return duration < timeout_duration_;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Exception checking BMS freshness: %s", e.what());
      return false;
    }
  }

  void timer_callback()
  {
    try {
      auto msg = sensor_msgs::msg::BatteryState();
      
      // Header with current timestamp
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = this->get_parameter("frame_id").as_string();
      
      // Check if BMS data is current
      if (is_bms_data_fresh() && last_bms_state_) {
        // Use BMS data as base
        msg = *last_bms_state_;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = this->get_parameter("frame_id").as_string();
        
        // Override with PZEM sensor data if available
        BatteryReadings local_readings = read_sensor_data();
        if (local_readings.valid) {
          msg.voltage = local_readings.voltage;
          msg.current = local_readings.current;
          
          RCLCPP_DEBUG(this->get_logger(), 
            "Using PZEM readings: V=%.2fV, I=%.3fA, P=%.2fW", 
            local_readings.voltage, local_readings.current, local_readings.power);
        }
        
      } else {
        // No BMS data available - do not publish anything
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "BMS data stale (%.2fs), not publishing battery state", 
          (this->get_clock()->now() - last_bms_update_).seconds());
        return; // Exit without publishing
      }
      
      // Publish the message
      battery_pub_->publish(msg);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in timer callback: %s", e.what());
    }
  }

  // Member variables
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // PZEM sensor for local voltage/current readings
  std::unique_ptr<PzemSensor> pzem_sensor_;
  
  // Status tracking
  std::optional<sensor_msgs::msg::BatteryState> last_bms_state_;
  rclcpp::Time last_bms_update_;
  
  // Configuration parameters
  double timeout_duration_;
  double publish_rate_;
};

int main(int argc, char * argv[])
{
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