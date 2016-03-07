#include "motors.h"
#include "io.h"
#include "sensors.h"
#include "state.h"
#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>

namespace io {
    const MotorPowers POWER_FULL = (MotorPowers){{255, 255, 255, 255}};

    static MotorPowers powerRelative(MotorPowers powers) {
        MotorPowers ret;
        for(int i = 0; i < 4; i++) {
            uint16_t power = (uint16_t)powers.powers[i] * (uint16_t)state::speed;
            ret.powers[i] = (uint8_t)(power / 0xff);
        }
        return ret;
    }

    void motorSet(uint8_t directions, MotorPowers powers) {
        powers = powerRelative(powers);
        for(int i = 0; i < 4; i++) {
            if(directions & (1 << i)) {
                forward(i, powers.powers[i]);
            } else {
                backward(i, powers.powers[i]);
            }
        }
    }

    MotorPowers adjustedMotorPowers(uint16_t front_weight, uint16_t back_weight) {
        int64_t _max = INT64_MIN;
        int64_t _min = INT64_MAX;
        for(uint8_t i = 0; i < 4; i++) {
            int64_t pos = abs(motor_positions[i] * (i < 2 ? front_weight : back_weight));
            if(pos > _max) {
                _max = pos;
            }
            if(pos < _min) {
                _min = pos;
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
            uint32_t pow;
            if(_max > abs(motor_positions[i])) {
                pow = high_pow;
            } else {
                pow = low_pow;
            }
            pow *= i < 2 ? front_weight : back_weight;
            pow /= max(front_weight, back_weight);
            ret.powers[i] = pow;
        }
        return ret;
    }
}
