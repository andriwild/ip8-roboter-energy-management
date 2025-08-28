#include "pzem_sensor.hpp"
#include <iostream>
#include <errno.h>
#include <unistd.h>

PzemSensor::PzemSensor(const std::string& device, int baud_rate, uint16_t slave_id) : ctx_(nullptr)
    , device_(device)
    , baud_rate_(baud_rate)
    , slave_id_(slave_id)
    , connected_(false)
{}

PzemSensor::~PzemSensor() {
    disconnect();
}

bool PzemSensor::initialize() {
    if (connected_) {
        return true;
    }

    // Create modbus:  baud: 9600, no parity (N), 8 data bits, 2 stop bits
    ctx_ = modbus_new_rtu(device_.c_str(), baud_rate_, 'N', 8, 2);
    if (ctx_ == nullptr) {
        std::cerr << "PZEM: Unable to create modbus context" << std::endl;
        return false;
    }

    // Connect to device
    if (modbus_connect(ctx_) == -1) {
        std::cerr << "PZEM: Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    modbus_set_slave(ctx_, slave_id_);
    sleep(1); 

    // Setup current range to 50A
    if (!setup_current_range()) {
        std::cerr << "PZEM: Failed to setup current range" << std::endl;
        disconnect();
        return false;
    }

    connected_ = true;
    std::cout << "PZEM: Successfully connected to " << device_ << std::endl;
    return true;
}

bool PzemSensor::setup_current_range(uint16_t range) {
    if (ctx_ == nullptr) {
        return false;
    }

    uint16_t current_value;
    int read_result = modbus_read_registers(ctx_, CURRENT_RANGE_REGISTER, 1, &current_value);
    if (read_result == -1) {
        std::cerr << "PZEM: Failed to read current range register: " 
                  << modbus_strerror(errno) << std::endl;
        return false;
    }

    std::cout << "PZEM: Current range register value: " << current_value << std::endl;

    if (current_value != range) {
        int write_result = modbus_write_register(ctx_, CURRENT_RANGE_REGISTER, range);
        if (write_result == -1) {
            std::cerr << "PZEM: Failed to set current range: " 
                      << modbus_strerror(errno) << std::endl;
            return false;
        }
        std::cout << "PZEM: Successfully set current range to 50A" << std::endl;
    } else {
        std::cout << "PZEM: Current range already set to 50A" << std::endl;
    }

    return true;
}

BatteryReadings PzemSensor::read() {
    BatteryReadings readings = {0.0f, 0.0f, 0.0f, 0.0f, false};

    if (!connected_ || ctx_ == nullptr) {
        return readings;
    }

    uint16_t regs[6];
    int rc = modbus_read_input_registers(ctx_, 0, 6, regs);
    
    if (rc == -1) {
        std::cerr << "PZEM: Read failed: " << modbus_strerror(errno) << std::endl;
        return readings;
    }

    readings.voltage = regs[0] * 0.01f;  // Register 0: Voltage (0.01V resolution)
    readings.current = regs[1] * 0.01f;  // Register 1: Current (0.01A resolution)
    
    // Power: 32-bit value stored in registers 2 and 3 (little-endian)
    readings.power = (((uint32_t)regs[3] << 16) | regs[2]) * 0.1f;  // 0.1W resolution
    
    // Energy: 32-bit value stored in registers 4 and 5 (little-endian)
    readings.energy = (((uint32_t)regs[5] << 16) | regs[4]);  // 1Wh resolution
    
    readings.valid = true;
    return readings;
}

void PzemSensor::disconnect() {
    if (ctx_ != nullptr) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}