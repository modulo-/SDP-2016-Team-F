#include "llcmd.h"
#include "sensors.h"
#include "io.h"
#include "motors.h"
#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>

#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_LEFT   2
#define MOTOR_BACK_RIGHT  3
#define MOTOR_GRABBERS    4

// Defines the forward/backward combinations for a movement command.
// The least significant 4 bits store this, with 0 being backward and 1 being
// forward for the corresponding motor ID.
#define MOVE_RIGHT ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_RIGHT))
#define MOVE_LEFT  ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_LEFT))
#define MOVE_CC    ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_RIGHT))
#define MOVE_CW    ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_LEFT))

#define cmdArg(offset, type) ((type *)(cmds + cmd_at + (offset)))

namespace llcmd {
    const uint8_t WAIT           = 0x00;
    const uint8_t BRAKE          = 0x01;
    const uint8_t GRABBER_OPEN   = 0x02;
    const uint8_t GRABBER_CLOSE  = 0x03;
    const uint8_t LED            = 0x07;
    const uint8_t STRAIT         = 0x08;
    const uint8_t SPIN           = 0x09;
    const uint8_t HOLD_SPIN      = 0x0a;
    const uint8_t NOP            = 0x7f;

    const uint8_t FLAG_UNINTERRUPTABLE = 0x80;

    const size_t cmd_cap = 256;
    uint8_t cmds[cmd_cap];
    size_t cmd_len = 0;
    size_t cmd_at = 0;
    unsigned long last_time = 0;

    static size_t argLen(uint8_t cmd) {
        if((cmd & 0x78) == 0x00) {
            return sizeof(uint16_t);
        }
        switch(cmd & 0x7f) {
        case STRAIT:
        case SPIN:
        case HOLD_SPIN:
            return sizeof(int16_t);
        default:
            return 0;
        }
    }

    // Checks if there are any commands left in the queue.
    // Also returns false if there is a command in the queue, but it has
    // insufficient arguments.
    bool idle() {
        return cmd_at == cmd_len || argLen(cmds[cmd_at]) + cmd_at >= cmd_len;
    }

    void run() {
        if(idle()) {
            return;
        }
        unsigned long new_time = millis();
        unsigned long delta = new_time - last_time;
        last_time = new_time;
        // If it's a time command (8bx000 0xxx)
        if((cmds[cmd_at] & 0x78) == 0x00) {
            uint16_t *data = cmdArg(1, uint16_t);
            if((cmds[cmd_at] & 0x7f) == BRAKE) {
                for(uint8_t i = 0; i < 4; i++) {
                    if(io::motor_positions[i] < -3) {
                        io::forward(i, 0x7f);
                    } else if(io::motor_positions[i] > 3) {
                        io::backward(i, 0x7f);
                    } else {
                        io::stop(i);
                    }
                }
            }
            if(delta >= *data) {
                *data = 0;
                finish(true);
            } else {
                *data -= delta;
            }
        } else if((cmds[cmd_at] & 0x7f) == STRAIT) {
            int16_t data = *cmdArg(1, int16_t);
            if(io::dist() >= abs(data)) {
                finish(true);
            } else {
                io::motorSet(
                    data > 0 ? MOVE_RIGHT : MOVE_LEFT,
                    io::adjustedMotorPowers());
            }
        } else if((cmds[cmd_at] & 0x7f) == SPIN) {
            int16_t data = *cmdArg(1, int16_t);
            if(io::rotDist() >= abs(data)) {
                finish(true);
            }
        }
    }

    void finish(bool advance) {
        switch(cmds[cmd_at] & 0x7f) {
        case HOLD_SPIN:
            io::stop(MOTOR_GRABBERS);
        case BRAKE:
        case STRAIT:
        case SPIN:
            io::stop(MOTOR_FRONT_LEFT);
            io::stop(MOTOR_FRONT_RIGHT);
            io::stop(MOTOR_BACK_LEFT);
            io::stop(MOTOR_BACK_RIGHT);
            break;
        case LED:
            digitalWrite(13, LOW);
            break;
        case GRABBER_OPEN:
        case GRABBER_CLOSE:
            io::stop(MOTOR_GRABBERS);
            break;
        }
        cmd_at += 1 + argLen(cmds[cmd_at]);
        if(advance) {
            start();
        }
    }

    void start() {
        for(int i = 0; i < 4; i++) {
            io::motor_positions[i] = 0;
        }
        if(idle()) {
            // Should really be stopped already, but hey, let's make sure.
            io::stopAll();
            return;
        }
        switch(cmds[cmd_at] & 0x7f) {
        case GRABBER_OPEN:
            io::forward(MOTOR_GRABBERS, 50);
            break;
        case GRABBER_CLOSE:
            io::backward(MOTOR_GRABBERS, 50);
            break;
        case HOLD_SPIN:
            io::backward(MOTOR_GRABBERS, 10);
        case SPIN:
            io::motorSet(*cmdArg(1, int16_t) > 0 ? MOVE_CW : MOVE_CC,
                io::POWER_FULL);
            break;
        case NOP:
            cmd_at++;
            start();
            break;
        case LED:
            digitalWrite(13, HIGH);
            break;
        }
        last_time = millis();
    }

    size_t uninterruptableChainLen() {
        size_t at = cmd_at;
        while(true) {
            if(at >= cmd_len) {
                return at - cmd_at;
            } else if((cmds[at] & FLAG_UNINTERRUPTABLE) == 0) {
                return at - cmd_at;
            }
            at += 1 + argLen(cmds[at]);
        }
    }
}
