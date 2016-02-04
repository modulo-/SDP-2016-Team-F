#include "comms.h"
#include "io.h"
#include "sensors.h"
#include "hlcmd.h"
#include "llcmd.h"
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

namespace comms {
    const char DEVICEID = '1';

    void process(const void *data, size_t len) {
        hlcmd::process(data, len);
    }
}

void setup() {
    io::init();
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    if(!comms::init("60", "~~~")) {
        digitalWrite(13, HIGH);
    }
    comms::send("rawr!", 'd', 5);
}

void loop() {
    comms::poll();
    io::poll();
    llcmd::run();
}
