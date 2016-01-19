#include <base64.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main() {
    char *ret = NULL;
    size_t len;
    encode_base64("test", 4, &ret, &len);
    assert(len == 6);
    assert(strncmp(ret, "dGVzdA", 6) == 0);
    char *ret2 = NULL;
    size_t len2;
    decode_base64(ret, len, &ret2, &len2);
    assert(len2 == 4);
    assert(strncmp(ret2, "test", 4) == 0);
    printf("base64 OK\n");
    return 0;
}
