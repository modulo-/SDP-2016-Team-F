#include <stdint.h>
#pragma once

namespace io {
    extern int64_t motor_positions[4];

    void poll();
    uint64_t rotDist();
    uint64_t dist();
}
