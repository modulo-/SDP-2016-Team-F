#include "sensors.h"
#include <stdint.h>
#include <Wire.h>

#define ROTARY_BOARD_I2C 5
#define ROTARY_COUNT 6

namespace io {
    int64_t motor_positions[4] = {0};

    void poll() {
        Wire.requestFrom(ROTARY_BOARD_I2C, ROTARY_COUNT);
        for(uint8_t i = 0; i < 4; i++) {
            motor_positions[i] += (int8_t)Wire.read();
        }
        Wire.read();
        Wire.read();
    }

    static uint64_t sumMotors() {
        uint64_t total = 0;
        for(uint8_t i = 0; i < 4; i++) {
            total += abs(motor_positions[i]);
        }
        return total;
    }

    uint64_t rotDist() {
        // Determined through totally scientific trial-and-error.
        return (sumMotors() * 20) / 1;
    }

    uint64_t dist() {
        // Determined through totally scientific trial-and-error.
        return (sumMotors() * 667) / 1000;
    }
}
