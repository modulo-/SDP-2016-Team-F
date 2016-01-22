#include "SDPArduino.h"
#include "comms.h"
#include <Wire.h>

namespace comms {
    const char DEVICEID = '1';

    void process(void *data, size_t len) {
        free(data);
    }
}

int i = 0;

void setup() {
    SDPsetup();
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    if(!comms::init("60", "~~~")) {
        digitalWrite(13, HIGH);
    }
}

void loop() {
    comms::poll();
}
