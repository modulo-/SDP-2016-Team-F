
namespace comms {

// The PANID used.
static const char * const PANID = "6810";
// The encryption key.
static const char * const ENCKEY = "3327bdbaaf48c59410fb5c4115777f26";
// The device "id" ('1' or '2').
extern const char DEVICEID;

// Initializes the comms system.
//
// Takes strings for the frequency, and the control character sequence.
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
bool init(const char *freq, const char *control);

// Checks for any recieved packets. If any are recieved, process will be
// called.
//
// Should run with little else to delay it in the main loop.
void poll();

// Sends a packet of data to a target.
void send(void *data, char target, size_t len);

// Processes recieved data.
//
// This is NOT defined by the comms library, but must be defined in the core
// arduino code.
void process(void *data, size_t len);

}
