// Library: 
// https://libmodbus.org/reference/#rtu-context
//
// Sensor protocoll datesheet: 
// https://thesunpays.com/downloads/files/Battery%20SOC%20meters/PZEM-003%20017User%20Manual(MEDC300V).pdf

// Bild and run: 
// g++ -std=c++17 client.cpp -lmodbus -o client
// ./client

#include <iostream>
#include <errno.h>
#include <modbus/modbus.h>
#include <unistd.h>
#include <csignal>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

const uint16_t SLAVE_ID = 1;
const uint16_t CURRENT_RANGE = 1; // 50A
const uint16_t CURRENT_RANGE_REGISTER = 0x0003;
const int POLLING_DELAY = 200; // ms

const std::string OUT_DIR = "/home/administrator/peacefair";


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

std::string getFormattedDateTime() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::tm tm_now = *std::localtime(&time_t_now);
    
    std::stringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

std::int64_t getUnixTimestampMs() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

int main(int argc, char* argv[]) {

    // Mode: default = CSV, optional --console to output to stdout
    bool csvMode = false;
    if (argc > 1 && std::string(argv[1]) == "--csv") {
        csvMode = true;
    }

    std::ofstream csvFile;
    if (csvMode) {
        std::string filename = OUT_DIR + "/data_" + getFormattedDateTime() + ".csv";

        csvFile.open(filename);
        if (!csvFile) {
            std::cerr << "Failed to open file " << filename << std::endl;
            return 1;
        }

        // write csv header
        csvFile << "timestamp,voltage,current,power,energy,wh" << std::endl;
    }

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
        int rc = modbus_read_input_registers(ctx, 0, 6, regs);
        if (rc == -1) {
            std::cerr << "Read failed: " << modbus_strerror(errno) << std::endl;
        } else {
            float u = regs[0] * 0.01; // voltage
            float i = regs[1] * 0.01; // current
            float p = (((uint32_t)regs[3] << 16) | regs[2]) * 0.1; // power
            float e = (((uint32_t)regs[5] << 16) | regs[4]); // energy

            if (csvMode) {
               csvFile << getUnixTimestampMs() << "," 
                       << u << ","
                       << i << ","
                       << p << ","
                       << e << ","
                       << std::endl;

            } else {
                std::cout << "Voltage: " << u << " V" << std::endl
                          << "Current: " << i << " A" << std::endl
                          << "Power:   " << p << " W" << std::endl
                          << "Energy:  " << e << " Wh" << std::endl
                          << std::endl;

            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(POLLING_DELAY));
    }

    if (csvFile.is_open()) csvFile.close();

    modbus_close(ctx);
    modbus_free(ctx);
    return EXIT_SUCCESS;
}