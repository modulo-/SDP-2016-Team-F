#include "hlcmd.h"
#include "llcmd.h"
#include "comms.h"
#include <stddef.h>
#include <string.h>
#include <Arduino.h>
#include <math.h>

#define cmdArg(cmd, offset, type) ((type *)((cmd) + (offset)))
#define ROTATION_EPSILON 1700

namespace hlcmd {
    const uint8_t WAIT          = 0x00;
    const uint8_t BRAKE         = 0x01;
    const uint8_t STRAIT        = 0x02;
    const uint8_t SPIN          = 0x03;
    const uint8_t KICK          = 0x04;
    const uint8_t MV            = 0x05;
    const uint8_t GRABBER_OPEN  = 0x06;
    const uint8_t GRABBER_CLOSE = 0x07;
    const uint8_t HOLD_SPIN     = 0x08;
    const uint8_t SPD_SET       = 0x09;
    const uint8_t ARC           = 0x0a;

    static uint16_t kickTimeFromDist(uint16_t dist) {
        if(dist >= 1500) {
            return 350;
        } else {
            uint32_t tmp = 9 * dist;
            tmp /= 250;
            return 170 + tmp;
        }
    }

    // Returns the length (command byte + argument lengths) required to validly
    // read the given HL command.
    size_t cmdRequiredLen(const uint8_t *cmd) {
        switch(*cmd) {
        case WAIT:
        case BRAKE:
        case STRAIT:
        case SPIN:
        case HOLD_SPIN:
        case KICK:
            return 3;
        case MV:
            return 13;
        case SPD_SET:
            return 2;
        case ARC:
            return 5;
        case GRABBER_OPEN:
        case GRABBER_CLOSE:
        default:
            return 1;
        }
    }

    // Returns the length of the low-level commands generated by the given HL
    // command.
    size_t cmdCompiledLen(const uint8_t *cmd) {
        switch(*cmd) {
        case WAIT:
        case BRAKE:
            // 1-to-1 mapping
            return 3;
        case GRABBER_OPEN:
        case KICK:
            // 1. Force grabbers
            // 2. Open grabbers partly
            // 3. Kick
            // 4. Open grabbers fully
            // 5. Interruptable NOP.
            return 13;
        case GRABBER_CLOSE:
            // 1. Uninterruptable slow close
            // 2. Uninterruptable slow open
            // 3. Uninterruptable fast close
            // 4. Uninterruptable slow close
            // 5. Interruptable NOP.
            return 13;
        case ARC:
            // 1. LL arc (7 bytes)
            // 2. Brake (3 bytes)
            return 10;
        case STRAIT:
        case HOLD_SPIN:
        case SPIN:
            // An implicit 100ms brake
            return 6;
        case MV:
            // 1. Spin
            // 2. Brake
            // 3. Strait
            // 4. Brake
            // 5. Spin
            // 6. Brake
            return 18;
        case SPD_SET:
            return 2;
        default:
            return 0;
        }
    }

    static void compileMV(const uint8_t *in, uint8_t *out) {
        int32_t x = *cmdArg(in, 7, int16_t) - *cmdArg(in, 1, int16_t);
        int32_t y = *cmdArg(in, 9, int16_t) - *cmdArg(in, 3, int16_t);
        int32_t angle_strt = *cmdArg(in, 5, int16_t);
        int32_t angle_end = *cmdArg(in, 11, int16_t);
        int32_t mv_angle;
        if(x == 0 && y > 0) {
            mv_angle = 16200;
        } else if(x == 0) {
            mv_angle = 5400;
        } else {
            mv_angle = -atan2f(float(y), float(x)) * 10800 / M_PI;
        }
        mv_angle = (((mv_angle - angle_strt) % 21600) + 21600) % 21600;
        int16_t dist = (int16_t)sqrt(x*x + y*y);
        if(mv_angle > 10800) {
            mv_angle -= 21600;
        }
        if(mv_angle > 5400) {
            dist = -dist;
            mv_angle -= 10800;
        } else if(mv_angle < -5400) {
            dist = -dist;
            mv_angle += 10800;
        }
        int16_t face_angle = (((angle_end - mv_angle - angle_strt) % 21600)
            + 21600) % 21600;
        if(face_angle > 10800) {
            face_angle -= 21600;
        }
        // 1. Spin
        out[0] = llcmd::SPIN;
        *((int16_t *)(out + 1)) = mv_angle;
        // 2. Brake
        out[3] = llcmd::BRAKE;
        *((int16_t *)(out + 4)) = 100;
        // 3. Strait
        out[6] = llcmd::STRAIT;
        *((int16_t *)(out + 7)) = dist;
        // 4. Brake
        out[9] = llcmd::BRAKE;
        *((int16_t *)(out + 10)) = 100;
        // 5. Spin
        out[12] = llcmd::SPIN;
        *((int16_t *)(out + 13)) = face_angle;
        // 6. Brake
        out[15] = llcmd::BRAKE;
        *((int16_t *)(out + 16)) = 100;
    }

    // Compiles a command from in to out.
    void cmdCompile(const uint8_t *in, uint8_t *out) {
        int16_t tmp;
        switch(*in) {
        case WAIT:
            *out = llcmd::WAIT;
            memcpy(out + 1, in + 1, 2);
            break;
        case BRAKE:
            *out = llcmd::BRAKE;
            memcpy(out + 1, in + 1, 2);
            break;
        case ARC:
            out[0] = llcmd::ARC;
            memcpy(out + 1, in + 1, 2);
            // Wheel distance = 130mm
            // => Moving on an arc of radius r means the forward wheels move on arc
            // with radius r-65 and the outer on arc with radius r+65.
            // If the total turning angle is d/r radians, the inner wheel movement is
            // (d/r)*(r-65) and the outer (d/r)*(r+65).
            //
            // => The ratio of the two (inner / outer) is:
            // ((d/r)*(r-65))/((d/r)*(r+65)) = (r-65)/(r+65)
            tmp = *((int16_t *)(in + 3));
            if(tmp > 65) {
                *((uint16_t *)(out + 3)) = tmp - 65;
                *((uint16_t *)(out + 5)) = tmp + 65;
            } else if(tmp < -65) {
                *((uint16_t *)(out + 3)) = -tmp + 65;
                *((uint16_t *)(out + 5)) = -tmp - 65;
            } else {
                *((uint16_t *)(out + 3)) = 1;
                *((uint16_t *)(out + 5)) = 1;
            }
            out[7] = llcmd::BRAKE;
            *((uint16_t *)(out + 8)) = 100;
            break;
        case STRAIT:
            out[0] = llcmd::STRAIT;
            memcpy(out + 1, in + 1, 2);
            out[3] = llcmd::BRAKE;
            *((uint16_t *)(out + 4)) = 100;
            break;
        case KICK:
            out[0] = llcmd::GRABBER_FORCE | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 1)) = 100;
            out[3] = llcmd::GRABBER_OPEN | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 4)) = 300;
            out[6] = llcmd::KICK | llcmd::FLAG_UNINTERRUPTABLE;
            memcpy(out + 7, in + 1, 2);
            out[9] = llcmd::GRABBER_OPEN | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 10)) = 600;
            out[12] = llcmd::NOP;
            break;
        case HOLD_SPIN:
            out[0] = llcmd::HOLD_SPIN;
            memcpy(out + 1, in + 1, 2);
            out[3] = llcmd::BRAKE;
            *((uint16_t *)(out + 4)) = 100;
            break;
        case SPIN:
            out[0] = llcmd::SPIN;
            tmp = *cmdArg(in, 1, int16_t);
            if(tmp > 0 && tmp < ROTATION_EPSILON) {
                tmp = ROTATION_EPSILON;
            } else if(tmp < 0 && tmp > -ROTATION_EPSILON) {
                tmp = -ROTATION_EPSILON;
            }
            *((int16_t *)(out + 1)) = tmp;
            out[3] = llcmd::BRAKE;
            *((uint16_t *)(out + 4)) = 100;
            break;
        case MV:
            compileMV(in, out);
            break;
        case SPD_SET:
            out[0] = llcmd::SPD_SET;
            out[1] = in[1];
            break;
        case GRABBER_OPEN:
            out[0] = llcmd::GRABBER_OPEN | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 1)) = 600;
            out[3] = llcmd::NOP;
            break;
        case GRABBER_CLOSE:
            out[0] = llcmd::GRABBER_CLOSE | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 1)) = 400;
            out[3] = llcmd::GRABBER_OPEN | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 4)) = 100;
            out[6] = llcmd::GRABBER_FORCE | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 7)) = 300;
            out[9] = llcmd::GRABBER_CLOSE | llcmd::FLAG_UNINTERRUPTABLE;
            *((uint16_t *)(out + 10)) = 200;
            out[12] = llcmd::NOP;
            break;
        }
    }

    void process(const void *data, size_t len) {
        // Step 1: Find the sequence on uninterruptable commands, and move them
        // to the start of the command buffer. Interrupt interruptable
        // commands.
        if(llcmd::idle()) {
            llcmd::cmd_len = 0;
        } else {
            llcmd::cmd_len = llcmd::uninterruptableChainLen();
            if(llcmd::cmd_len == 0) {
                llcmd::finish(false);
            } else {
                memmove(llcmd::cmds, llcmd::cmds + llcmd::cmd_at,
                    llcmd::cmd_len);
            }
        }
        llcmd::cmd_at = 0;
        // Step 2: Compile the hl commands, one-by-one, into ll commands.
        const uint8_t *cmds = (const uint8_t *)data;
        size_t at = 0;
        while(at < len && at + cmdRequiredLen(&cmds[at]) <= len) {
            size_t len = cmdCompiledLen(&cmds[at]);
            if(llcmd::cmd_len + len > llcmd::cmd_cap) {
                const char *err = "Command buffer overflow. Discarding.";
                comms::send(err, 'e', strlen(err));
                break;
            }
            uint8_t bytecode[len];
            cmdCompile(&cmds[at], bytecode);
            memcpy(llcmd::cmds + llcmd::cmd_len, bytecode, len);
            llcmd::cmd_len += len;
            at += cmdRequiredLen(&cmds[at]);
        }
        llcmd::start();
    }
}
