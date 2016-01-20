# Radio Comms Protocol Specification

Data recieved over the RF link is split into packets, with each packet being a
single line of ASCII text.

## Packet Types

There are two types of packets, data packets and acknowledgement packets.

Data packets when recieved and verified MUST be replied to with an
acknowledgement packet. Acknowledgement packets MUST NOT be replied to.

## Data Packet

A data packet has the following format:

`<target><source><data><checksum>\n`

`<target>` is one of `1`, `2`, or `c`, standing for group 1**1**'s robot, group
1**2**'s robot and the **c**ontroller respectively. A device which is not the
target of the packet must ignore it.

`<source>` follows the same convention and specifies the target the
acknowledgement packet should have.

`<data>` is the base64-encoded binary packet.

`<checksum>` is two bytes containing calculated to verify all preceding data
(including `<target>` and `<source>`). If the verification failes, the packet
MUST be ignored.

## Acknowledgement Packet

An acknowledgement packet has the following format:

`<target>$<checksum>\n`

The `$` signals that the packet is an acknowledgement, since this cannot occur
in valid base64. `<target>` SHOULD have sent a packet with the same checksum as
transmitted in `<checksum>`, and should mark this as acknowledged. Incorrect
acknowledgements MUST be ignored.

## Base64

The standard index table for base64 is used, with character 62 being `+` and 63
being `/`. When encoding, the input is padded with zero-bits to reach a
multiple of 6 bits. When decoding, excess bits are discarded.

### Checksums

The checksum's first and second bytes is the xor of the base64-values of all
even and odd characters respectively:

```
def checksum(str):
    b0 = 0x00
    b1 = 0x00
    for(i = 0; i < len(str); i++):
        if i % 2 == 0:
            b0 ^= base64val(str[i])
        else:
            b1 ^= base64val(str[i])
    return base64char(b0) + base64char(b1)
```
