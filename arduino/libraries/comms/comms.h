#pragma once
#include <stddef.h>

namespace comms {

// The device "id" ('1' or '2').
//
// This is NOT defined by the comms library, but must be defined in the core
// arduino code.
extern const char DEVICEID;

// Initializes the comms system.
//
// Takes strings for the channel, and the control character sequence.
//
// Also sets the panid and encryption keys.
//
// Returns true if the initialization is successful.
//
// Example usage:
//
// if(!comms::init("60", "~~~")) {
//     // DO PANIC
// }
bool init(const char *chan, const char *control);

// Checks for any recieved packets. If any are recieved, process will be
// called.
//
// Should run with little else to delay it in the main loop.
void poll();

// Sends a packet of data to a target.
void send(const void *data, char target, size_t len);

// Processes recieved data.
//
// This is NOT defined by the comms library, but must be defined in the core
// arduino code.
//
// This DOES NOT recieves ownership of `data`, and *must not* assume it to be
// persistant.
void process(const void *data, size_t len);

}
