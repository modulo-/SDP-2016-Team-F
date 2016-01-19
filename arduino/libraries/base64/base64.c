#include "base64.h"

uint8_t base64_val(char c) {
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

char base64_char(uint8_t val) {
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

bool base64_valid(char *ptr, size_t len) {
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

struct checksum base64_checksum(char *ptr, size_t len) {
    uint8_t sums[2] = {0x00, 0x00};
    for(size_t i = 0; i < len; i++) {
        sums[i % 2] ^= base64_val(ptr[i]);
    }
    return (struct checksum){{base64_char(sums[0]), base64_char(sums[1])}};
}

void decode_base64(char *ptr, size_t len, char **dec_ptr, size_t *dec_len) {
    // TODO
}

void encode_base64(char *ptr, size_t len, char **enc_ptr, size_t *enc_len) {
    // TODO
}
