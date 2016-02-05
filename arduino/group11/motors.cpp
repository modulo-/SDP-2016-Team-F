#include "motors.h"
#include "io.h"
#include "sensors.h"
#include <stdint.h>
#include <stdlib.h>

namespace io {
    const MotorPowers POWER_FULL = (MotorPowers){{255, 255, 255, 255}};

    void motorSet(uint8_t directions, MotorPowers powers) {
        for(int i = 0; i < 4; i++) {
            if(directions & (1 << i)) {
                forward(i, powers.powers[i]);
            } else {
                backward(i, powers.powers[i]);
            }
        }
    }

    MotorPowers adjustedMotorPowers() {
        int64_t _max = INT64_MIN;
        int64_t _min = INT64_MAX;
        for(uint8_t i = 0; i < 4; i++) {
            if(abs(motor_positions[i]) > _max) {
                _max = abs(motor_positions[i]);
            }
            if(abs(motor_positions[i]) < _min) {
                _min = abs(motor_positions[i]);
            }
        }
        int64_t delta = _max - _min;
        if(delta <= 1) {
            return POWER_FULL;
        }
        uint8_t high_pow = 255;
        uint8_t low_pow;
        if(delta < 4) {
            low_pow = 230;
        } else if(delta < 10) {
            low_pow = 204;
        } else if(delta < 20) {
            low_pow = 178;
        } else {
            low_pow = 0x7f;
        }
        MotorPowers ret;
        for(uint8_t i = 0; i < 4; i++) {
            if(_max > abs(motor_positions[i])) {
                ret.powers[i] = high_pow;
            } else {
                ret.powers[i] = low_pow;
            }
        }
        return ret;
    }
}
