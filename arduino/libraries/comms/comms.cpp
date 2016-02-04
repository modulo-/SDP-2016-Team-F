#include "comms.h"
#include "base64.h"

#include <stddef.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>

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
    if(tryCmd(control, NULL))
        return false;
    if(tryCmd(NULL, "ATEE1"))
        return false;
    if(tryCmd(NULL, "ATAC"))
        return false;
    if(tryCmd("ATEK", ENCKEY))
        return false;
    if(tryCmd("ATID", PANID))
        return false;
    if(tryCmd("ATCN", chan))
        return false;
    if(tryCmd(NULL, "ATAC"))
        return false;
    if(tryCmd(NULL, "ATDN"))
        return false;
    return true;
}

// Processes the full packet in buf.
static void processPacket() {
    // A valid message has at least 5 characters (not counting ACKs).
    if(buflen < 6) {
        buflen = 0;
        return;
    // Different target
    } else if(buf[0] != DEVICEID) {
        buflen = 0;
        return;
    // Invalid base64
    } else if(!base64::valid(buf, buflen - 2)) {
        buflen = 0;
        return;
    }
    base64::Checksum chksum = base64::checksum(buf, buflen - 4);
    // Invalid checksum
    if(strncmp(chksum.sum, buf + buflen - 4, 2)) {
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
    size_t decoded_len = base64::decLen(buflen - 6);
    char decoded[decoded_len];
    base64::decode(buf + 2, buflen - 6, decoded);
    buflen = 0;
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

void send(const void *data, char target, size_t len) {
    // Note that as of right now, the arduino will IGNORE acknowledgement
    // packets.
    size_t enclen = base64::encLen(len);
    char encoded[enclen];
    base64::encode((char *)data, len, encoded);
    Serial.print(target);
    Serial.print(DEVICEID);
    Serial.print(encoded);
    base64::Checksum chksum = base64::checksum(encoded, enclen);
    chksum.sum[0] = base64::chr(
        base64::val(chksum.sum[0]) ^ base64::val(target));
    chksum.sum[1] = base64::chr(
        base64::val(chksum.sum[1]) ^ base64::val(DEVICEID));
    Serial.print(chksum.sum[0]);
    Serial.print(chksum.sum[1]);
    Serial.println("");
}

}
