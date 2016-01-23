#include "SDPArduino.h"
#include "comms.h"
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

uint8_t *cmds;
size_t cmd_len;
size_t cmd_at;

unsigned long last_time;

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
        digitalWrite(13, HIGH);
        motorForward(3, 100);
        break;
    case 0x02:
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
        if(delta >= *data) {
            *data = 0;
            if(cmds[cmd_at] == 0x00) {
                digitalWrite(13, LOW);
            } else {
                digitalWrite(13, LOW);
                motorStop(3);
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
}

void loop() {
    comms::poll();
    cmdRun();
}
