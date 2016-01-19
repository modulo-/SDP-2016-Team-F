#include <base64.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

struct test {
    char *enc;
    char *dec;
};

int main() {
    char *buf = NULL;
    size_t buflen;
    struct test tests[] = {
        {"dGVzdA", "test"},
        {"JCQlKkBhc2RmWU9JS0xKOg", "$$%*@asdfYOIKLJ:"},
        {}
    };
    struct test *at = tests;
    while(at->enc) {
        base64Encode(at->dec, strlen(at->dec), &buf, &buflen);
        assert(base64Valid(at->enc, strlen(at->enc)));
        assert(buflen == strlen(at->enc));
        assert(strncmp(buf, at->enc, buflen) == 0);
        base64Decode(at->enc, strlen(at->enc), &buf, &buflen);
        assert(buflen == strlen(at->dec));
        assert(strncmp(buf, at->dec, buflen) == 0);
        at++;
    }
    assert(!base64Valid("$", 1));
    assert(!base64Valid("^", 1));
    assert(!base64Valid("%", 1));
    assert(!base64Valid("*", 1));
    assert(!base64Valid("\n", 1));
    printf("base64 OK\n");
    return 0;
}
