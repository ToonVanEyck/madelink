'''
Madelink Custom UART Protocol Decoder

This decoder handles a custom UART-based protocol with the following structure:
- First 3 bytes: Header
- Followed by COBS (Consistent Overhead Byte Stuffing) encoded payloads
- Each COBS payload is terminated by a zero byte

The decoder will:
1. Identify and display the 3-byte / 1-byte header
2. Collect COBS-encoded payload bytes until a zero delimiter
3. Decode the COBS payload and display both raw and decoded data

COBS encoding removes zero bytes from data streams by using overhead bytes
to indicate where zeros should be inserted during decoding.
'''

from .pd import Decoder
