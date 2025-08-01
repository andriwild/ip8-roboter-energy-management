#ifndef PZEM_SENSOR_HPP
#define PZEM_SENSOR_HPP

#include <memory>
#include <string>
#include <modbus/modbus.h>

struct BatteryReadings {
    float voltage;    // V
    float current;    // A
    float power;      // W
    float energy;     // Wh
    bool valid;       // Data validity flag
};

class PzemSensor {
public:
    PzemSensor(const std::string& device = "/dev/ttyUSB0", 
               int baud_rate = 9600,
               uint16_t slave_id = 1);
    ~PzemSensor();
    
    // Initialize connection and setup sensor
    bool initialize();
    
    // Read current sensor values
    BatteryReadings read();
    
    // Check if sensor is connected
    bool is_connected() const { return connected_; }
    
    // Cleanup connection
    void disconnect();

private:
    bool setup_current_range(uint16_t range = 1); // 1 = 50A range
    
    modbus_t* ctx_;
    std::string device_;
    int baud_rate_;
    uint16_t slave_id_;
    bool connected_;
    
    static const uint16_t CURRENT_RANGE_REGISTER = 0x0003;
    static const uint16_t DEFAULT_CURRENT_RANGE = 1; // 50A
};

#endif // PZEM_SENSOR_HPP