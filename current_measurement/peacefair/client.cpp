// Library: 
// https://libmodbus.org/reference/#rtu-context
//
// Sensor protocoll datesheet: 
// https://thesunpays.com/downloads/files/Battery%20SOC%20meters/PZEM-003%20017User%20Manual(MEDC300V).pdf

#include <iostream>
#include <errno.h>
#include <modbus.h>
#include <unistd.h>
#include <csignal>


const uint16_t SLAVE_ID = 1;
const uint16_t CURRENT_RANGE = 1; // 50A
const uint16_t CURRENT_RANGE_REGISTER = 0x0003;
const float POLLING_RATE = 20; // Hz (10Hz = 100ms)

int setupCurrentRange(modbus_t *ctx, uint16_t range = CURRENT_RANGE) {
    if (ctx == NULL) {
        std::cerr << "Modbus context must be initialized!" << std::endl;
        return EXIT_FAILURE;
    }

    // Read current range
    uint16_t value;
    int read_success = modbus_read_registers(ctx, CURRENT_RANGE_REGISTER, 1, &value);
    if (read_success == -1) {
        std::cerr << "Current register read failed: " << modbus_strerror(errno) << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Current register: " << value << std::endl;
    }

    // Set current range to 50A
    int success = modbus_write_register(ctx, CURRENT_RANGE_REGISTER, CURRENT_RANGE);
    if (success == -1) {
        std::cerr << "Failed to set current range: " << modbus_strerror(errno) << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Successfully set current range to 50A" << std::endl;
    }

    return EXIT_SUCCESS;
}


int main() {
    // baud: 9600, no parity (N), 8 data bits, 2 stop bits
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 2);
    if (ctx == NULL) {
        std::cerr << "Unable to create the modbus context" << std::endl;
        return EXIT_FAILURE;
    }

    if (modbus_connect(ctx) == -1) {
        std::cerr  << "Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_close(ctx);
        modbus_free(ctx);
        return EXIT_FAILURE;
    }

    modbus_set_slave(ctx, SLAVE_ID);
    sleep(1);

    //int success = setupCurrentRange(ctx);
    //if (success == -1) {
    //    modbus_close(ctx);
    //    modbus_free(ctx);
    //    return EXIT_SUCCESS;
    //}

    while (true) {
        uint16_t regs[6];
        int rc = modbus_read_input_registers(ctx, 1, 6, regs);
        if (rc == -1) {
            std::cerr << "Read failed: " << modbus_strerror(errno) << std::endl;
        } else {
            std::cout << "Voltage: " << regs[0] * 0.01 << " V\n"
                      << "Current: " << regs[1] * 0.01 << " A\n"
                      << "Power:   "
                      << (((uint32_t)regs[3] << 16) | regs[2]) * 0.1 << " W\n"
                      << "Energy:  " << (((uint32_t)regs[5] << 16) | regs[4])
                      << " Wh" << std::endl
                      << std::endl;
        }
    }

    modbus_close(ctx);
    modbus_free(ctx);
    return EXIT_SUCCESS;
}