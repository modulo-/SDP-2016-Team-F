#ifndef _BASE64_H
#define _BASE64_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Since fixed-size arrays cannot be returned directly.
struct checksum {
    char sum[2];
};

// Gets the 6-bit value associated with a base64 character.
//
// c MUST be a valid base64 character. (Verify with `base64_valid(&c, 1)`)
uint8_t base64_val(char c);
// Gets the character associated with a 6-bit value.
//
// 0 <= val < 64 MUST be satisfied.
char base64_char(uint8_t val);
// Verifies that a sequence of text is valid base64.
bool base64_valid(char *ptr, size_t len);
// Calculates the 2-character checkum of some text in base64.
struct checksum base64_checksum(char *ptr, size_t len);
// Decodes the base64-sequence pointed to by ptr of length len. Stores the
// result in *dec_ptr, (re-)allocating as necessary. Stores the decoded length
// in dec_len.
//
// Example usage:
//
// char *base64 = "<some valid base64>";
// char *dec = NULL;
// size_t len;
// base64_decode(base64, strlen(base64), &dec, &len);
void decode_base64(char *ptr, size_t len, char **dec_ptr, size_t *dec_len);
// Encodes base64. See decode_base64 for the workings.
void encode_base64(char *ptr, size_t len, char **enc_ptr, size_t *enc_len);

#endif
