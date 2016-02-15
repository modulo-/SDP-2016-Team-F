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
// result in the first decLen(len) bytes of dec_ptr. The caller must ensure
// that this is a safe memory operation.
void decode(const char *ptr, size_t len, char *dec_ptr);
// Encodes base64. See encode for the workings.
//
// The first encLen(len) bytes of enc_ptr will be set to the values. The caller
// must ensure that this is a safe memory operation.
void encode(const char *ptr, size_t len, char *enc_ptr);
// Returns the length of an base64 encoding of data of length len.
size_t encLen(size_t len);
// Returns the length of an base64 decoded data for input of length len.
size_t decLen(size_t len);

}
