#include "io.h"
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#define MOTOR_BOARD_I2C 4

namespace io {
    void init() {
        pinMode(2, INPUT);
        pinMode(3, OUTPUT);
        pinMode(4, INPUT);
        pinMode(5, OUTPUT);
        pinMode(6, OUTPUT);
        pinMode(7, INPUT);
        pinMode(8, OUTPUT);
        pinMode(9, OUTPUT);
        pinMode(10, INPUT);
        pinMode(11, INPUT);
        pinMode(12, INPUT);
        pinMode(13, INPUT);
        pinMode(A0, INPUT);
        pinMode(A1, INPUT);
        pinMode(A2, INPUT);
        pinMode(A3, INPUT);
        digitalWrite(8, HIGH);
        Serial.begin(115200);
        Wire.begin();
    }

    void forward(uint8_t motor, uint8_t power) {
        uint8_t send[2] = {motor << 5 | 28, power};
        Wire.beginTransmission(MOTOR_BOARD_I2C);
        Wire.write(send, 2);
        Wire.endTransmission();
    }

    void backward(uint8_t motor, uint8_t power) {
        uint8_t send[2] = {motor << 5 | 30, power};
        Wire.beginTransmission(MOTOR_BOARD_I2C);
        Wire.write(send, 2);
        Wire.endTransmission();
    }

    void stop(uint8_t motor) {
        uint8_t send[1] = {motor << 5 | 16};
        Wire.beginTransmission(MOTOR_BOARD_I2C);
        Wire.write(send, 1);
        Wire.endTransmission();
    }

    void stopAll() {
        uint8_t send[1] = {1};
        Wire.beginTransmission(MOTOR_BOARD_I2C);
        Wire.write(send, 1);
        Wire.endTransmission();
    }

    void brake(uint8_t motor) {
        uint8_t send[1] = {motor << 5 | 18};
        Wire.beginTransmission(MOTOR_BOARD_I2C);
        Wire.write(send, 1);
        Wire.endTransmission();
    }

    void brakeAll() {
        brake(0);
        brake(1);
        brake(2);
        brake(3);
        brake(4);
        brake(5);
    }
}
