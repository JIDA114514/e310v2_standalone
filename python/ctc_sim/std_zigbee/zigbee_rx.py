#!/usr/bin/env python3
# coding=utf-8

import sys, time, zmq, signal, numpy as np
from gr_zigbee import gr_zigbee as gr_block

CHIP_MAP = [
    "11011001110000110101001000101110",  # 0x0
    "11101101100111000011010100100010",  # 0x1
    "00101110110110011100001101010010",  # 0x2
    "00100010111011011001110000110101",  # 0x3
    "01010010001011101101100111000011",  # 0x4
    "00110101001000101110110110011100",  # 0x5
    "11000011010100100010111011011001",  # 0x6
    "10011100001101010010001011101101",  # 0x7
    "10001100100101100000011101111011",  # 0x8
    "10111000110010010110000001110111",  # 0x9
    "01111011100011001001011000000111",  # 0xA
    "01110111101110001100100101100000",  # 0xB
    "00000111011110111000110010010110",  # 0xC
    "01100000011101111011100011001001",  # 0xD
    "10010110000001110111101110001100",  # 0xE
    "11001001011000000111011110111000",  # 0xF
]
PREAMBLE_BYTES = 4
SFD = 0xA7
KNOWN_FRAME_LEN = 14  # bytes: 4 preamble + 1 SFD + 1 len + 6 payload + 2 FCS

class ZMQSubscriber:
    def __init__(self, addr="tcp://127.0.0.1:55556"):
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.connect(addr)
        self.socket.setsockopt(zmq.SUBSCRIBE, b'')
    def iswaiting(self): return self.socket.poll(10)
    def read(self): return self.socket.recv()
    def close(self): self.socket.close()

def unpack_bytes_to_chips(data):
    chips = []
    for byte in data:
        for i in range(8):
            chips.append("1" if (byte >> i) & 1 else "0")
    return "".join(chips)

def chips_to_symbols(chips):
    symbols = []
    usable = (len(chips) // 32) * 32
    for i in range(0, usable, 32):
        chunk = chips[i:i+32]
        best_s, best_d = 0, 33
        for s, ref in enumerate(CHIP_MAP):
            d = sum(1 for a, b in zip(chunk, ref) if a != b)
            if d < best_d: best_d, best_s = d, s
        symbols.append((best_s, best_d))
    return symbols

def symbols_to_bits(symbols):
    return "".join(f"{s:04b}" for s, _ in symbols)

def bits_to_bytes_lsb(bit_str):
    data = []
    for i in range(0, len(bit_str) - len(bit_str) % 8, 8):
        v = 0
        for idx, ch in enumerate(bit_str[i:i+8]):
            if ch == "1": v |= 1 << idx
        data.append(v)
    return data

def find_preamble(data, known_len=KNOWN_FRAME_LEN):
    """Search for preamble + SFD, return frame of known_len bytes."""
    preamble = [0x00] * PREAMBLE_BYTES
    for i in range(0, len(data) - known_len + 1):
        if data[i:i+PREAMBLE_BYTES] == preamble and data[i+PREAMBLE_BYTES] == SFD:
            return data[i:i+known_len], i
    return None, -1

# ── Main ──────────────────────────────────────────────────
gr_block_obj = gr_block()
gr_block_obj.start()
gr_block_obj.set_zigbee_channel(11)
print(f"RX freq: {gr_block_obj.get_freq()/1e6:.1f} MHz, "
      f"sr: {gr_block_obj.get_sample_rate()/1e6:.1f} MHz")

zmq_sub = ZMQSubscriber("tcp://127.0.0.1:55556")
zmq_msgs = 0
last_report = time.time()
chip_buf = ""
MAX_CHIPS = 9600

try:
    while True:
        raw = None
        if zmq_sub.iswaiting() != 0:
            raw = zmq_sub.read()
            zmq_msgs += 1

        if raw and len(raw) > 0:
            chips = unpack_bytes_to_chips(raw)
            chip_buf += chips
            if len(chip_buf) > MAX_CHIPS:
                chip_buf = chip_buf[-MAX_CHIPS:]

            # Try frame detection
            if len(chip_buf) >= 32 * (PREAMBLE_BYTES + 2):
                syms = chips_to_symbols(chip_buf)
                if syms:
                    bits = symbols_to_bits(syms)
                    data = bits_to_bytes_lsb(bits)
                    frame, pos = find_preamble(data)
                    if frame is not None:
                        # Print raw chips at preamble
                        chip_pos = pos * 32
                        print(f"\n=== PREAMBLE FOUND at byte {pos} ===")
                        print(f"Chips around preamble: {chip_buf[chip_pos:chip_pos+64]}")
                        print(f"Symbol distances: {[d for _,d in syms[pos:pos+KNOWN_FRAME_LEN//2]]}")
                        print(f"Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")
                        print(f"  preamble: {' '.join(f'{b:02X}' for b in frame[:4])}")
                        print(f"  SFD: {frame[4]:02X}")
                        print(f"  length: {frame[5]}")
                        print(f"  payload+FCS: {' '.join(f'{b:02X}' for b in frame[6:])}")
                        # Advance past this frame
                        end_byte = pos + KNOWN_FRAME_LEN
                        chip_buf = chip_buf[end_byte * 32:]

        if time.time() - last_report >= 2.0:
            if zmq_msgs > 0:
                # Print a few raw chips for debugging
                preview = chip_buf[:100] if chip_buf else "(empty)"
                print(f"[msgs:{zmq_msgs} chips:{len(chip_buf)} raw:{preview}]")
            last_report = time.time()

except KeyboardInterrupt:
    zmq_sub.close()
    gr_block_obj.stop()
    gr_block_obj.wait()
    print("safe exit")
