#include "base64.h"

namespace base64 {

uint8_t val(char c) {
    if(c >= 'A' && c <= 'Z') {
        return c - 'A';
    } else if(c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    } else if(c >= '0' && c <= '9') {
        return c - '0' + 52;
    } else if(c == '+') {
        return 62;
    } else if(c == '/') {
        return 63;
    } else {
        return 0xff;
    }
}

char chr(uint8_t val) {
    if(val < 26) {
        return val + 'A';
    } else if(val < 52) {
        return val - 26 + 'a';
    } else if(val < 62) {
        return val - 52 + '0';
    } else if(val == 62) {
        return '+';
    } else if(val == 63) {
        return '/';
    } else {
        return '?';
    }
}

bool valid(const char *ptr, size_t len) {
    for(size_t i = 0; i < len; i++) {
        char c = ptr[i];
        if(!((c >= 'A' && c <= 'Z')
                || (c >= 'a' && c <= 'z')
                || (c >= '0' && c <= '9')
                || c == '+'
                || c == '/')) {
            return false;
        }
    }
    return true;
}

Checksum checksum(const char *ptr, size_t len) {
    uint8_t sums[2] = {0x00, 0x00};
    for(size_t i = 0; i < len; i++) {
        sums[i % 2] ^= val(ptr[i]);
    }
    return (Checksum){{chr(sums[0]), chr(sums[1])}};
}

void decode(const char *ptr, size_t len, char *dec_ptr) {
    uint8_t buf = 0x00;
    uint8_t bits_in_buf = 0;
    for(size_t i = 0; i < len; i++) {
        uint8_t tmp;
        switch(bits_in_buf) {
        case 0:
            buf = val(ptr[i]);
            bits_in_buf = 6;
            break;
        case 2:
            *dec_ptr = (buf << 6) | val(ptr[i]);
            dec_ptr++;
            buf = 0x00;
            bits_in_buf = 0;
            break;
        case 4:
            tmp = val(ptr[i]);
            *dec_ptr = (buf << 4) | tmp >> 2;
            dec_ptr++;
            buf = tmp & 0x03;
            bits_in_buf = 2;
            break;
        case 6:
            tmp = val(ptr[i]);
            *dec_ptr = (buf << 2) | tmp >> 4;
            dec_ptr++;
            buf = tmp & 0x0f;
            bits_in_buf = 4;
            break;
        }
    }
}

void encode(const char *ptr, size_t len, char *enc_ptr) {
    uint8_t *ptr2 = (uint8_t *)ptr;
    uint8_t buf = 0x00;
    uint8_t bits_in_buf = 0;
    for(size_t i = 0; i < len; i++) {
        switch(bits_in_buf) {
        case 0:
            *enc_ptr = chr(ptr2[i] >> 2);
            enc_ptr++;
            buf = ptr2[i] & 0x03;
            bits_in_buf = 2;
            break;
        case 2:
            *enc_ptr = chr(buf << 4 | ptr2[i] >> 4);
            enc_ptr++;
            buf = ptr2[i] & 0x0f;
            bits_in_buf = 4;
            break;
        case 4:
            enc_ptr[0] = chr(buf << 2 | ptr2[i] >> 6);
            enc_ptr[1] = chr(ptr2[i] & 0x3f);
            enc_ptr += 2;
            buf = 0x00;
            bits_in_buf = 0;
            break;
        }
    }
    switch(bits_in_buf) {
        case 2:
            *enc_ptr = chr(buf << 4);
            break;
        case 4:
            *enc_ptr = chr(buf << 2);
            break;
    }
}

size_t encLen(size_t len) {
    size_t tmp = len * 4;
    return tmp / 3 + (tmp % 3 == 0 ? 0 : 1);
}

size_t decLen(size_t len) {
    return (len * 3) / 4;
}

}
