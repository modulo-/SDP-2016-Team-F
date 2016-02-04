#pragma once
#include <stdint.h>
#include <stddef.h>

namespace llcmd {
    extern const uint8_t WAIT;
    extern const uint8_t KICKER_RETRACT;
    extern const uint8_t KICKER_EXTEND;
    extern const uint8_t BRAKE;
    extern const uint8_t STRAIT;
    extern const uint8_t SPIN;
    extern const uint8_t NOP;

    extern const uint8_t FLAG_UNINTERRUPTABLE;
    extern const size_t cmd_cap;
    extern uint8_t cmds[];
    extern size_t cmd_len;
    extern size_t cmd_at;

    bool idle();
    void run();
    void start();
    void finish(bool advance);
    size_t uninterruptableChainLen();
}
