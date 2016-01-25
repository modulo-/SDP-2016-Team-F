#include "SDPArduino.h"
#include "comms.h"
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

uint8_t *cmds;
size_t cmd_len = 0;
size_t cmd_at = 0;
uint8_t buf[250];
uint8_t buf_len = 0;
uint8_t buf_at = 0;
uint16_t buf_print_delay = 100;

unsigned long last_time;
unsigned long start_time;

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
        motorBackward(2, 100);
        motorForward(3, 100);
        break;
    case 0x04:
        motorForward(0, 100);
        motorBackward(1, 100);
        motorForward(2, 100);
        motorBackward(3, 100);
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
    // 0xf0: Set buffer length
    case 0xf0:
        buf_len = cmds[cmd_at + 1];
        cmd_at += 2;
        cmdAdvance();
        break;
    // 0xf1: Set delay
    case 0xf1:
        buf_print_delay = *((uint16_t *)(cmds + cmd_at + 1));
        cmd_at += 3;
        cmdAdvance();
        break;
    // 0xf2: Start writing buffer
    case 0xf2:
        buf_at = 0;
        start_time = millis();
        break;
    // 0x80 - 0x8a: Set buffer part (25 bytes long each)
    case 0x80:
    case 0x81:
    case 0x82:
    case 0x83:
    case 0x84:
    case 0x85:
    case 0x86:
    case 0x87:
    case 0x88:
    case 0x89:
    case 0x8a:
        uint8_t *ptr = buf + (cmds[cmd_at] - 0x80) * 25;
        memcpy(ptr, cmds + cmd_at + 1, 25);
        cmd_at += 26;
        cmdAdvance();
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
    case 0xf2:
        if(new_time >= buf_at * buf_print_delay + start_time) {
            Wire.beginTransmission(0x45);
            Wire.write(buf[buf_at++]);
            Wire.endTransmission();
            if(buf_at == buf_len) {
                cmd_at++;
                cmdAdvance();
            }
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
