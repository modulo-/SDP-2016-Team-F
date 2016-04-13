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
        uint64_t s = sumMotors();
        if(s < 50) {
            return (max(17, s) - 17) * 35 / 25;
        } else if(s < 75) {
            return (s - 50) * 35 / 25 + 45;
        } else if(s < 100) {
            return (s - 75) * 23 / 25 + 77;
        } else if(s < 150) {
            return (s - 100) * 40 / 50 + 100;
        } else if(s < 200) {
            return (s - 150) * 32 / 50 + 140;
        } else if(s < 250) {
            return (s - 200) * 38 / 50 + 172;
        } else if(s < 300) {
            return (s - 250) * 30 / 50 + 210;
        } else if(s < 400) {
            return (s - 300) * 65 / 100 + 240;
        } else if(s < 500) {
            return (s - 400) * 50 / 100 + 305;
        } else if(s < 750) {
            return (s - 500) * 175 / 250 + 355;
        } else {
            return (s - 750) * 170 / 250 + 530;
        }
    }
}
