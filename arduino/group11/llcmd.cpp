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
#define MOTOR_KICKER1     4
#define MOTOR_KICKER2     5

// Defines the forward/backward combinations for a movement command.
// The least significant 4 bits store this, with 0 being backward and 1 being
// forward for the corresponding motor ID.
#define MOVE_RIGHT ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_LEFT))
#define MOVE_LEFT  ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_RIGHT))
#define MOVE_CC    ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_RIGHT))
#define MOVE_CW    ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_LEFT))

// All `_T` commands take a 16-bit millisecond paramter, and execute for the
// specified time.
#define CMD_WAIT_T           0x00
#define CMD_KICKER_RETRACT_T 0x01
#define CMD_KICKER_EXTEND_T  0x02
#define CMD_MV_RIGHT_T       0x03
#define CMD_MV_LEFT_T        0x04
#define CMD_SPIN_CC_T        0x05
#define CMD_SPIN_CW_T        0x06
#define CMD_BRAKE_T          0x07
// CMD_MV_STRAIT takes a 16-bit signed integer, representing the number of
// millimeters to move strait. Positive amounts denote moving to the right,
// negative amounts moving to the left.
#define CMD_MV_STRAIT        0x80
// CMD_SPIN takes a 16-bit signed integer, representing the number of minutes
// to spin clockwise.
#define CMD_SPIN             0x81
// CMD_KICK takes a 16-bit signed integer, representing the number of
// millimeters to kick the ball.
#define CMD_KICK             0x82
// CMD_MV executes complex movement. Takes 3 16-bit arguments: x (positive:
// right, negative: left) and y (positive: forwards, negative: backwards)
// displacement in mm, and an angle to face at the end (postive: clockwise,
// negative: anticlockwise) relative to the current direction, in minutes.
#define CMD_MV               0x83

#define cmdArg(offset, type) ((type *)(cmds + cmd_at + (offset)))

namespace llcmd {
    const size_t cmd_cap = 256;
    uint8_t cmds[cmd_cap];
    size_t cmd_len = 0;
    size_t cmd_at = 0;
    unsigned long last_time = 0;

    static size_t argLen(uint8_t cmd) {
        if(cmd & 0xf0 == 0x00) {
            return sizeof(uint16_t);
        }
        switch(cmd) {
        case CMD_MV_STRAIT:
        case CMD_SPIN:
        case CMD_KICK:
            return sizeof(int16_t);
        case CMD_MV:
            return sizeof(int16_t) * 3;
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
        // If it's a time command (0x0*):
        if(cmds[cmd_at] & 0xf0 == 0x00) {
            uint16_t *data = cmdArg(1, uint16_t);
            if(cmds[cmd_at] == CMD_BRAKE_T) {
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
        } else if(cmds[cmd_at] == CMD_MV_STRAIT) {
            int16_t data = *cmdArg(1, int16_t);
            if(io::dist() >= abs(data)) {
                finish(true);
            } else {
                io::motorSet(
                    data > 0 ? MOVE_RIGHT : MOVE_LEFT,
                    io::adjustedMotorPowers());
            }
        } else if(cmds[cmd_at] == CMD_SPIN) {
            int16_t data = *cmdArg(1, int16_t);
            if(io::rotDist() >= abs(data)) {
                finish(true);
            }
        }
    }

    void finish(bool advance) {
        switch(cmds[cmd_at]) {
        case CMD_KICKER_RETRACT_T:
        case CMD_KICKER_EXTEND_T:
            io::stop(MOTOR_KICKER1);
            io::stop(MOTOR_KICKER2);
            break;
        case CMD_MV_RIGHT_T:
        case CMD_MV_LEFT_T:
        case CMD_SPIN_CC_T:
        case CMD_SPIN_CW_T:
        case CMD_BRAKE_T:
        case CMD_MV_STRAIT:
        case CMD_SPIN:
            io::stop(MOTOR_FRONT_LEFT);
            io::stop(MOTOR_FRONT_RIGHT);
            io::stop(MOTOR_BACK_LEFT);
            io::stop(MOTOR_BACK_RIGHT);
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
        switch(cmds[cmd_at]) {
        case CMD_KICKER_RETRACT_T:
            io::forward(MOTOR_KICKER1, 100);
            io::forward(MOTOR_KICKER2, 100);
            break;
        case CMD_KICKER_EXTEND_T:
            io::backward(MOTOR_KICKER1, 100);
            io::backward(MOTOR_KICKER2, 100);
            break;
        case CMD_MV_RIGHT_T:
            io::motorSet(MOVE_RIGHT, io::POWER_FULL);
            break;
        case CMD_MV_LEFT_T:
            io::motorSet(MOVE_LEFT, io::POWER_FULL);
            break;
        case CMD_SPIN_CC_T:
            io::motorSet(MOVE_CC, io::POWER_FULL);
            break;
        case CMD_SPIN_CW_T:
            io::motorSet(MOVE_CW, io::POWER_FULL);
            break;
        case CMD_SPIN:
            io::motorSet(*cmdArg(1, int16_t) > 0 ? MOVE_CW : MOVE_CC,
                io::POWER_FULL);
            break;
        }
        last_time = millis();
    }
}
