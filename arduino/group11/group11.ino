#include "SDPArduino.h"
#include "comms.h"
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

uint8_t *cmds;
size_t cmd_len = 0;
size_t cmd_at = 0;

unsigned long last_time = 0;

void cmdAdvance() {
    if(cmd_at == cmd_len) {
        motorAllStop();
        return;
    }
    switch(cmds[cmd_at]) {
    case 0x00:
        digitalWrite(13, HIGH);
        break;
    case 0x01:
        motorForward(4, 100);
        motorForward(5, 100);
        break;
    case 0x02:
        motorBackward(4, 100);
        motorBackward(5, 100);
        break;
    case 0x03:
        motorBackward(0, 100);
        motorForward(1, 100);
        motorBackward(2, 98);
        motorForward(3, 98);
        break;
    case 0x04:
        motorForward(0, 100);
        motorBackward(1, 100);
        motorForward(2, 98);
        motorBackward(3, 98);
        break;
    case 0x05:
        motorForward(0, 100);
        motorBackward(1, 100);
        motorBackward(2, 100);
        motorForward(3, 100);
        break;
    case 0x06:
        motorBackward(0, 100);
        motorForward(1, 100);
        motorForward(2, 100);
        motorBackward(3, 100);
        break;
    }
    last_time = millis();
}

void cmdRun() {
    if(cmd_at == cmd_len) {
        return;
    }
    uint16_t *data = (uint16_t *)(cmds + cmd_at + 1);
    unsigned long new_time = millis();
    unsigned long delta = new_time - last_time;
    last_time = new_time;
    switch(cmds[cmd_at]) {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
        if(delta >= *data) {
            *data = 0;
            switch(cmds[cmd_at]) {
            case 0x00:
                digitalWrite(13, LOW);
                break;
            case 0x01:
            case 0x02:
                motorStop(4);
                motorStop(5);
                break;
            case 0x03:
            case 0x04:
            case 0x05:
            case 0x06:
                motorStop(0);
                motorStop(1);
                motorStop(2);
                motorStop(3);
                break;
            }
            cmd_at += 3;
            cmdAdvance();
        } else {
            *data -= delta;
        }
        break;
    default:
        cmd_at++;
        cmdAdvance();
        break;
    }
}

namespace comms {
    const char DEVICEID = '1';

    void process(void *data, size_t len) {
        // TODO finish the current command.
        free(cmds);
        cmds = (uint8_t *)data;
        cmd_len = len;
        cmd_at = 0;
        cmdAdvance();
    }
}

int i = 0;

void setup() {
    SDPsetup();
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    if(!comms::init("60", "~~~")) {
//        digitalWrite(13, HIGH);
    }
    comms::send("hello", 'c', 5);
}

void loop() {
    comms::poll();
    cmdRun();
}
