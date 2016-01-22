import comms

comms.init("COM18", "67", "~~~", listen=True)


data = "1234567890"

# beware: octal
comms.send(b"d\0\2\12", 2)
comms.send(b"d\1\0\3abc", 2)
comms.send(b"d\2\3\3def", 2)
comms.send(b"d\3\6\3ghi", 2)
comms.send(b"d\4\11\1j", 2)

for i in range(5):
    print i
