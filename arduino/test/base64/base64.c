#include "base64.h"
#include <assert.h>
#include <stdio.h>

void main() {
    char *ret;
    size_t len;
    encode_base64("test", 4, &ret, &len);
    printf("%s\n", ret);
}
