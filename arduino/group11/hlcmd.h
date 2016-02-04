#pragma once
#include <stddef.h>
#include <stdint.h>


namespace hlcmd {
    extern const uint8_t WAIT;
    extern const uint8_t BRAKE;
    extern const uint8_t STRAIT;
    extern const uint8_t SPIN;
    extern const uint8_t KICK;
    extern const uint8_t MV;

    void process(const void *data, size_t len);
}
