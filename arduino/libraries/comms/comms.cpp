#include "comms.h"
#include "base64.h"

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <Arduino.h>

#define MAXBUF 64

namespace comms {

static char buf[MAXBUF];
static uint8_t buflen = 0;

// The PANID used.
static const char * const PANID = "6810";
// The encryption key.
static const char * const ENCKEY = "3327bdbaaf48c59410fb5c4115777f26";

static bool tryCmd(const char *prefix, const char *cmd) {
    if(prefix) {
        Serial.print(prefix);
    }
    if(cmd) {
        Serial.println(cmd);
    }
    return !Serial.find("OK");
}

bool init(const char *chan, const char *control) {
    Serial.setTimeout(5000);
    // Hooray for abusing short circuiting!
    if(tryCmd(control, NULL)
            || tryCmd(NULL, "ATEE1")
            || tryCmd(NULL, "ATAC")
            || tryCmd("ATEC", ENCKEY)
            || tryCmd("ATID", PANID)
            || tryCmd("ATCN", chan)
            || tryCmd(NULL, "ATAC")
            || tryCmd(NULL, "ATDN")) {
        return false;
    }
    return true;
}

// Processes the full packet in buf.
static void processPacket() {
    // A valid message has at least 5 characters (not counting ACKs).
    if(buflen < 5) {
        buflen = 0;
        return;
    // Different target
    } else if(buf[0] != DEVICEID) {
        buflen = 0;
        return;
    // Invalid base64
    } else if(!base64::valid(buf, buflen - 1)) {
        buflen = 0;
        return;
    }
    base64::Checksum chksum = base64::checksum(buf, buflen - 3);
    // Invalid checksum
    if(strncmp(chksum.sum, buf + buflen - 3, 2)) {
        buflen = 0;
        return;
    }
    // Everything ok!

    // Send ACK
    Serial.print(buf[1]);
    Serial.print('$');
    Serial.print(chksum.sum[0]);
    Serial.print(chksum.sum[1]);
    Serial.println("");
    // Decode
    char *decoded = NULL;
    size_t decoded_len;
    base64::decode(buf + 2, buflen - 5, &decoded, &decoded_len);
    process(decoded, decoded_len);
}

void poll() {
    int c;
    while((c = Serial.read()) != -1) {
        // Message too long. Drop the buffer.
        if(buflen >= MAXBUF) {
            buflen = 0;
        }
        buf[buflen++] = (char)c;
        // We have a packet seperator, check the packet.
        if(c == '\n') {
            processPacket();
        }
    }
}

void send(void *data, char target, size_t len) {
    // Note that as of right now, the arduino will IGNORE acknowledgement
    // packets.
    char *encoded = NULL;
    size_t enclen;
    base64::encode((char *)data, len, &encoded, &enclen);
    Serial.print(target);
    Serial.print(DEVICEID);
    Serial.print(encoded);
    base64::Checksum chksum = base64::checksum(encoded, enclen);
    Serial.print(chksum.sum[0]);
    Serial.print(chksum.sum[1]);
    Serial.println("");
    free(encoded);
}

}
