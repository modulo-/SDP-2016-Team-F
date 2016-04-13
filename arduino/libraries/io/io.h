#include <stdint.h>

#pragma once

namespace io {
    void init();
    void forward(uint8_t motor, uint8_t power);
    void backward(uint8_t motor, uint8_t power);
    void stop(uint8_t motor);
    void stopAll();
    void brake(uint8_t motor);
    void brakeAll();
}
