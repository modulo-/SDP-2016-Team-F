import base64

def encode(data):
    return base64.b64encode(data).rstrip('=')

def checksum(data):
    chk = [0, 0]
    for (i, chr) in enumerate(data):
        chk[i % 2] ^= value(chr)
    return ''.join(char(x) for x in chk)

def decode(data):
    if len(data) % 4 == 2:
        data += '=='
    elif len(data) % 4 == 3:
        data += '='
    return base64.b64decode(data)

def validchar(chr):
    return ((chr >= 'A' and chr <= 'Z')
        or (chr >= 'a' and chr <= 'z')
        or (chr >= '0' and chr <= '9')
        or chr == '+'
        or chr == '/')

def valid(data):
    all(map(validchar, data))

def value(chr):
    if chr >= 'A' and chr <= 'Z':
        return ord(chr) - ord('A')
    if chr >= 'a' and chr <= 'z':
        return ord(chr) - ord('a') + 26
    if chr >= '0' and chr <= '9':
        return ord(chr) - ord('0') + 52
    if chr == '+':
        return 62
    if chr == '/':
        return 63
    raise ValueError("Invalid b64 character!")

def char(val):
    if val < 26:
        return chr(val + ord('A'))
    if val < 52:
        return chr(val + ord('a') - 26)
    if val < 62:
        return chr(val + ord('0') - 52)
    if val == 62:
        return '+'
    if val == 63:
        return '/'
    raise ValueError("Invalid b64 value!")
