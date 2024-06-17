#ifndef LUNA_PERIPHERAL_DEF_H
#define LUNA_PERIPHERAL_DEF_H

#include <cstdint>
#include "luna_pin_def.h"

namespace stella::config {
    constexpr uint32_t PYRO_ACTIVATE_INTERVAL = 1000ul;

    constexpr auto HZ_TO_INTERVAL_MS          = [](const double FREQUENCY_HZ) -> uint32_t {
        return static_cast<uint32_t>(1000. / FREQUENCY_HZ);
    };

    constexpr auto INTERVAL_MS_TO_HZ = [](const uint32_t INTERVAL) -> uint32_t {
        return 1000 / INTERVAL;
    };

    constexpr uint8_t INA236_ADDRESS = 0x40;

    enum INA236ADCRange {
        RANGE_80MV = 0,
        RANGE_20MV = 1
    };

    constexpr unsigned long RFD900X_BAUD         = 460800;
    constexpr unsigned long UART_BAUD            = 115200;
    constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT     = 250ul;  // u-blox GPS comm timeout
    constexpr uint32_t SD_SPI_CLOCK_MHZ          = 20ul;   // 20 MHz
    constexpr size_t MESSAGE_BUFFER_SIZE         = 512ul;

    constexpr uint32_t TX_IDLE_INTERVAL          = HZ_TO_INTERVAL_MS(1);  // 1 Hz

    constexpr uint32_t LOG_IDLE_INTERVAL         = HZ_TO_INTERVAL_MS(1);   // 1 Hz

    constexpr uint32_t BUZZER_ON_INTERVAL        = 50ul;                    // 50 ms
    constexpr uint32_t BUZZER_IDLE_INTERVAL      = HZ_TO_INTERVAL_MS(1);    // 1 Hz

    constexpr auto BUZZER_OFF_INTERVAL           = [](const uint32_t BUZZER_TOTAL_INTERVAL) -> uint32_t {
        return BUZZER_TOTAL_INTERVAL - BUZZER_ON_INTERVAL;
    };
}  // namespace luna::config


#endif  //LUNA_PERIPHERAL_DEF_H
