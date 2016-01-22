#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

namespace base64 {

// Since fixed-size arrays cannot be returned directly.
typedef struct checksum {
    char sum[2];
} Checksum;

// Gets the 6-bit value associated with a base64 character.
//
// c MUST be a valid base64 character. (Verify with `base64_valid(&c, 1)`)
uint8_t val(char c);
// Gets the character associated with a 6-bit value.
//
// 0 <= val < 64 MUST be satisfied.
char chr(uint8_t val);
// Verifies that a sequence of text is valid base64.
bool valid(const char *ptr, size_t len);
// Calculates the 2-character checkum of some text in base64.
Checksum checksum(const char *ptr, size_t len);
// Decodes the base64-sequence pointed to by ptr of length len. Stores the
// result in *dec_ptr, (re-)allocating as necessary. Stores the decoded length
// in dec_len.
//
// Ownership of *dec_ptr is transferred to the caller and must be freed.
//
// Example usage:
//
// const char *base64 = "<some valid base64>";
// char *dec = NULL;
// size_t len;
// base64::decode(base64, strlen(base64), &dec, &len);
// // Do shit
// free(dec);
void decode(const char *ptr, size_t len, char **dec_ptr, size_t *dec_len);
// Encodes base64. See encode for the workings.
//
// *enc_ptr will be a null terminated string; the null character is NOT counted
// toward the returned length.
void encode(const char *ptr, size_t len, char **enc_ptr, size_t *enc_len);

}