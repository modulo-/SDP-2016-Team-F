#pragma once
#include <stdint.h>

namespace io {
    // Since fixed-size arrays cannot be returned directly.
    typedef struct motor_powers {
        uint8_t powers[4];
    } MotorPowers;

    extern const MotorPowers POWER_FULL;

    void motorSet(uint8_t direction, MotorPowers powers);
    MotorPowers adjustedMotorPowers();
}
