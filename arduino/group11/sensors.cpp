#include "sensors.h"
#include <stdint.h>
#include <Wire.h>
#include <Arduino.h>

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
        uint64_t s = sumMotors();
        if(s < 120) {
            return (max(30, s) - 30) * 30;
        } else if(s < 450) {
            return (s - 120) * 540 / 22 + 2700;
        } else if(s < 700) {
            return (s - 450) * 108 / 5 + 10800;
        } else {
            return (s - 700) * 18 + 16200;
        }
    }

    uint64_t dist() {
        // Determined through totally scientific trial-and-error.
        // TODO: TMP
        return sumMotors();
        //return (sumMotors() * 9) / 10;
    }
}
