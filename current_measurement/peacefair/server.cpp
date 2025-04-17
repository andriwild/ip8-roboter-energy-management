// start virtual ports
// socat -d -d pty,raw,echo=0 pty,raw,echo=0

// server.cpp
#include <modbus.h>
#include <modbus-rtu.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <iostream.h>

int main() {
    // 1) Create RTU context: /dev/pts/3, 9600-8-N-2
    modbus_t* ctx = modbus_new_rtu("/dev/pts/4", 9600, 'N', 8, 2);
    if (!ctx) {
        std::cout << "" << std::endl;
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return EXIT_FAILURE;
    }                                                  // :contentReference[oaicite:3]{index=3}

    // 2) (Optional) Set RS‑485 RTS mode if needed
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485); // :contentReference[oaicite:4]{index=4}

    // 3) Set server (slave) address
    modbus_set_slave(ctx, 1);

    // 4) Connect to the “wire”
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return EXIT_FAILURE;
    }

    // 5) Allocate register map
    modbus_mapping_t* mb_mapping = modbus_mapping_new(0, 0, 10, 0);
    if (!mb_mapping) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        return EXIT_FAILURE;
    }

    // 6) Initialize registers to some values
    for (int i = 0; i < 10; ++i) {
        mb_mapping->tab_registers[i] = i * 10;
    }

    // 7) Serve requests forever
    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
    while (true) {
        int rc = modbus_receive(ctx, query);
        if (rc > 0) {
            modbus_reply(ctx, query, rc, mb_mapping);
        } else if (rc == -1) {
            // Error or connection closed
            break;
        }
    }

    // 8) Clean up
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);
    return EXIT_SUCCESS;
}
