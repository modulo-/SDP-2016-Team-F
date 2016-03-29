#include "motors.h"
#include "io.h"
#include "sensors.h"
#include "state.h"
#include "comms.h"
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
            if(directions & (2 << i)) {
                forward(i + 1, powers.powers[i]);
            } else {
                backward(i + 1, powers.powers[i]);
            }
        }
    }

    MotorPowers adjustedMotorPowers(uint16_t front_weight, uint16_t back_weight) {
        int64_t front = (abs(motor_positions[0]) + abs(motor_positions[1]))
            * front_weight;
        int64_t back = (abs(motor_positions[2]) + abs(motor_positions[3]))
            * back_weight;
        int64_t delta = abs(front - back) / max(front_weight, back_weight);
        if(delta <= 1) {
            return POWER_FULL;
        }
        uint8_t high_pow = 255;
        uint8_t low_pow;
        if(delta < 2) {
            low_pow = 230;
        } else if(delta < 4) {
            low_pow = 204;
        } else if(delta < 10) {
            low_pow = 178;
        } else {
            low_pow = 0x7f;
        }
        MotorPowers ret;
        double front_multiplier = (double)front_weight /
            (double)max(front_weight, back_weight);
        double back_multiplier = (double)back_weight /
            (double)max(front_weight, back_weight);
        uint8_t front_pow;
        uint8_t back_pow;
        if(front < back) {
            digitalWrite(13, HIGH);
            front_pow = high_pow;
            back_pow = low_pow;
        } else {
            digitalWrite(13, LOW);
            front_pow = low_pow;
            back_pow = high_pow;
        }
        ret.powers[0] = front_pow * front_multiplier;
        ret.powers[1] = front_pow * front_multiplier;
        ret.powers[2] = back_pow * back_multiplier;
        ret.powers[3] = back_pow * back_multiplier;
        return ret;
    }
}
