#include "base64.h"

#include <stdlib.h>

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

// TODO: test
void decode_base64(char *ptr, size_t len, char **dec_ptr, size_t *dec_len) {
    *dec_len = (len * 3) / 4;
    *dec_ptr = realloc(*dec_ptr, sizeof(char) * *dec_len);
    uint8_t buf = 0x00;
    uint8_t bits_in_buf = 0;
    uint8_t *nextc = *dec_ptr;
    for(size_t i = 0; i < len; i++) {
        uint8_t tmp;
        switch(bits_in_buf) {
        case 0:
            buf = base64_val(ptr[i]);
            bits_in_buf = 6;
            break;
        case 2:
            *nextc = (buf << 6) | base64_val(ptr[i]);
            nextc++;
            buf = 0x00;
            bits_in_buf = 0;
            break;
        case 4:
            tmp = base64_val(ptr[i]);
            *nextc = (buf << 4) | tmp >> 2;
            nextc++;
            buf = tmp & 0x03;
            bits_in_buf = 2;
            break;
        case 6:
            tmp = base64_val(ptr[i]);
            *nextc = (buf << 2) | tmp >> 4;
            nextc++;
            buf = tmp & 0x0f;
            bits_in_buf = 4;
            break;
        }
    }
}

// TODO: test
void encode_base64(char *ptr, size_t len, char **enc_ptr, size_t *enc_len) {
    *enc_len = (len * 4);
    *enc_len = *enc_len / 3 + (*enc_len % 3 == 0 ? 0 : 1);
    *enc_ptr = realloc(*enc_ptr, sizeof(char) * (*enc_len + 1));
    (*enc_ptr)[*enc_len] = '\0';
    uint8_t buf = 0x00;
    uint8_t bits_in_buf = 0;
    char *nextc = *enc_ptr;
    for(size_t i = 0; i < len; i++) {
        switch(bits_in_buf) {
        case 0:
            *nextc = base64_char(ptr[i] >> 2);
            nextc++;
            buf = ptr[i] & 0x03;
            bits_in_buf = 2;
            break;
        case 2:
            *nextc = base64_char(buf << 4 | ptr[i] >> 4);
            nextc++;
            buf = ptr[i] & 0x0f;
            bits_in_buf = 4;
            break;
        case 4:
            nextc[0] = base64_char(buf << 2 | ptr[i] >> 6);
            nextc[1] = base64_char(ptr[i] & 0x3f);
            nextc += 2;
            buf = 0x00;
            bits_in_buf = 0;
            break;
        }
    }
}
