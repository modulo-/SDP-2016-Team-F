#include "SDPArduino.h"
#include "comms.h"
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_LEFT   2
#define MOTOR_BACK_RIGHT  3
#define MOTOR_KICKER1     4
#define MOTOR_KICKER2     5

// All `_T` commands take a 16-bit millisecond paramter, and execute for the
// specified time.
#define CMD_NOP_T            0x00
#define CMD_KICKER_RETRACT_T 0x01
#define CMD_KICKER_EXTEND_T  0x02
#define CMD_MV_RIGHT_T       0x03
#define CMD_MV_LEFT_T        0x04
#define CMD_SPIN_CC_T        0x05
#define CMD_SPIN_CW_T        0x06
// CMD_MV_STRAIT takes a 16-bit signed integer, representing the number of
// millimeters to move strait. Positive amounts denote moving to the right,
// negative amounts moving to the left.
#define CMD_MV_STRAIT        0x80
// CMD_SPIN takes a 16-bit signed integer, representing the number of degrees
// to spin clockwise.
#define CMD_SPIN             0x81
// CMD_KICK takes a 16-bit signed integer, representing the number of
// millimeters to kick the ball.
#define CMD_KICK             0x82
// CMD_MV executes complex movement. Takes 3 16-bit arguments: x (positive:
// right, negative: left) and y (positive: forwards, negative: backwards)
// displacement in mm, and an angle to face at the end (postive: clockwise,
// negative: anticlockwise) relative to the current direction, in degrees.
#define CMD_MV               0x83

// Defines the forward/backward combinations for a movement command.
// The least significant 4 bits store this, with 0 being backward and 1 being
// forward for the corresponding motor ID.
#define MOVE_LEFT  ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_LEFT))
#define MOVE_RIGHT ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_RIGHT))
#define MOVE_CC    ((1 << MOTOR_FRONT_LEFT)  | (1 << MOTOR_BACK_RIGHT))
#define MOVE_CW    ((1 << MOTOR_FRONT_RIGHT) | (1 << MOTOR_BACK_LEFT))

#define cmdArg(offset, type) ((type *)(cmds + cmd_at + (offset)))

// Since fixed-size arrays cannot be returned directly.
typedef struct motor_powers {
    uint8_t powers[4];
} MotorPowers;

#define POWER_FULL ((MotorPowers){{100, 100, 100, 100}})

uint8_t *cmds;
size_t cmd_len = 0;
size_t cmd_at = 0;
int64_t motor_positions[4] = {0};
unsigned long last_time = 0;

void motorSet(uint8_t directions, MotorPowers powers) {
    for(int i = 0; i < 4; i++) {
        if(directions & (1 << i)) {
            motorForward(i, powers.powers[i]);
        } else {
            motorBackward(i, powers.powers[i]);
        }
    }
}

int64_t dist(int64_t motor_positions[4]) {
    // TODO
    return 0;
}

MotorPowers adjustedMotorPowers() {
    // TODO
    return (MotorPowers){0};
}

size_t argLen(uint8_t cmd) {
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
bool cmdQueueEmpty() {
    return cmd_at == cmd_len || argLen(cmds[cmd_at]) + cmd_at >= cmd_len;
}

void cmdAdvance() {
    for(int i = 0; i < 4; i++) {
        motor_positions[i] = 0;
    }
    if(cmdQueueEmpty()) {
        // Should really be stopped already, but hey, let's make sure.
        motorAllStop();
        return;
    }
    switch(cmds[cmd_at]) {
    case CMD_KICKER_RETRACT_T:
        motorForward(MOTOR_KICKER1, 100);
        motorForward(MOTOR_KICKER2, 100);
        break;
    case CMD_KICKER_EXTEND_T:
        motorBackward(MOTOR_KICKER1, 100);
        motorBackward(MOTOR_KICKER2, 100);
        break;
    case CMD_MV_RIGHT_T:
        motorSet(MOVE_RIGHT, POWER_FULL);
        break;
    case CMD_MV_LEFT_T:
        motorSet(MOVE_LEFT, POWER_FULL);
        break;
    case CMD_SPIN_CC_T:
        motorSet(MOVE_CC, POWER_FULL);
        break;
    case CMD_SPIN_CW_T:
        motorSet(MOVE_CW, POWER_FULL);
        break;
    }
    last_time = millis();
}

void cmdFinish(bool advance) {
    switch(cmds[cmd_at]) {
    case CMD_KICKER_RETRACT_T:
    case CMD_KICKER_EXTEND_T:
        motorStop(MOTOR_KICKER1);
        motorStop(MOTOR_KICKER2);
        break;
    case CMD_MV_RIGHT_T:
    case CMD_MV_LEFT_T:
    case CMD_SPIN_CC_T:
    case CMD_SPIN_CW_T:
    case CMD_MV_STRAIT:
        motorStop(MOTOR_FRONT_LEFT);
        motorStop(MOTOR_FRONT_RIGHT);
        motorStop(MOTOR_BACK_LEFT);
        motorStop(MOTOR_BACK_RIGHT);
        break;
    }
    cmd_at += 1 + argLen(cmds[cmd_at]);
    if(advance) {
        cmdAdvance();
    }
}

void cmdRun() {
    if(cmdQueueEmpty()) {
        return;
    }
    unsigned long new_time = millis();
    unsigned long delta = new_time - last_time;
    last_time = new_time;
    // If it's a time command (0x0*):
    if(cmds[cmd_at] & 0xf0 == 0x00) {
        uint16_t *data = cmdArg(1, uint16_t);
        if(delta >= *data) {
            *data = 0;
            cmdFinish(true);
        } else {
            *data -= delta;
        }
    } else if(cmds[cmd_at] == CMD_MV_STRAIT) {
        int16_t data = *cmdArg(1, int16_t);
        if(abs(dist(motor_positions)) >= abs(data)) {
            cmdFinish(true);
        } else {
            motorSet(
                data > 0 ? MOVE_RIGHT : MOVE_LEFT,
                adjustedMotorPowers());
        }
    }
}

namespace comms {
    const char DEVICEID = '1';

    void process(void *data, size_t len) {
        if(!cmdQueueEmpty()) {
            // TODO: Some commands (e.g. kick) should not be aborted even if
            // a new order comes in. Figure something out. (Maybe even keep
            // kicking while moving?)
            cmdFinish(false);
        }
        free(cmds);
        cmds = (uint8_t *)data;
        cmd_len = len;
        cmd_at = 0;
        cmdAdvance();
    }
}

void pollSensors() {
    // TODO
}

void setup() {
    SDPsetup();
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    if(!comms::init("60", "~~~")) {
//        digitalWrite(13, HIGH);
    }
    comms::send("rawr!", 'd', 5);
}

void loop() {
    comms::poll();
    pollSensors();
    cmdRun();
}
