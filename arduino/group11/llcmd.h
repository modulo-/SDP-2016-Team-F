#pragma once
#include <stdint.h>
#include <stddef.h>

namespace llcmd {
    extern uint8_t *cmds;
    extern size_t cmd_len;
    extern size_t cmd_at;

    bool idle();
    void run();
    void start();
    void finish(bool advance);
}
