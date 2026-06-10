#!/usr/bin/env python3
# coding=utf-8
import sys, time, zmq, signal, numpy as np
from gr_zigbee import gr_zigbee as gr_block
from zigbee_mod import crc16_ccitt

CHIP_MAP = [
    "11011001110000110101001000101110",
    "11101101100111000011010100100010",
    "00101110110110011100001101010010",
    "00100010111011011001110000110101",
    "01010010001011101101100111000011",
    "00110101001000101110110110011100",
    "11000011010100100010111011011001",
    "10011100001101010010001011101101",
    "10001100100101100000011101111011",
    "10111000110010010110000001110111",
    "01111011100011001001011000000111",
    "01110111101110001100100101100000",
    "00000111011110111000110010010110",
    "01100000011101111011100011001001",
    "10010110000001110111101110001100",
    "11001001011000000111011110111000",
]
PREAMBLE_CHIPS = CHIP_MAP[0]  # "11011001110000110101001000101110"
PREAMBLE_BYTES = 4
SFD = 0xA7
KNOWN_FRAME_LEN = 14

class ZMQSubscriber:
    def __init__(self, addr="tcp://127.0.0.1:55556", hwm=20):
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, hwm)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.socket.connect(addr)

    def read_available(self, max_messages=200):
        messages = []
        if self.socket.poll(10) == 0:
            return messages
        while len(messages) < max_messages:
            try:
                messages.append(self.socket.recv(zmq.NOBLOCK))
            except zmq.Again:
                break
        return messages

    def close(self): return self.socket.close()

def unpack_bytes_to_chips(data):
    chips = []
    for byte in data:
        for i in range(8): chips.append("1" if (byte >> i) & 1 else "0")
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

def find_preamble(data):
    preamble = [0x00] * PREAMBLE_BYTES
    for i in range(0, len(data) - KNOWN_FRAME_LEN + 1):
        if data[i:i+PREAMBLE_BYTES] == preamble and data[i+PREAMBLE_BYTES] == SFD:
            return data[i:i+KNOWN_FRAME_LEN], i
    return None, -1


gr_block_obj = gr_block()
gr_block_obj.start()
gr_block_obj.set_zigbee_channel(11)
print(f"RX: {gr_block_obj.get_freq()/1e6:.1f} MHz "
      f"sr={gr_block_obj.get_sample_rate()/1e6:.1f} MHz")

zmq_sub = ZMQSubscriber()
zmq_msgs = 0
crc_ok_packets = 0
preamble_only_packets = 0
last_report = time.time()
last_clear = time.time()
chip_buf = ""
MAX_CHIPS = 9600

try:
    while True:
        raw_msgs = zmq_sub.read_available()
        if raw_msgs:
            zmq_msgs += len(raw_msgs)
            chips = "".join(unpack_bytes_to_chips(raw) for raw in raw_msgs if raw)
            chip_buf += chips
            if len(chip_buf) > MAX_CHIPS:
                chip_buf = chip_buf[-MAX_CHIPS:]

        # Clear stale noise buffer if no signal for a while
        if zmq_msgs > 0 and time.time() - last_clear > 3.0:
            chip_buf = ""
            last_clear = time.time()

        if len(chip_buf) >= 32 * (PREAMBLE_BYTES + 2):
            # Search for preamble chip pattern to find DSSS alignment
            pos = chip_buf.find(PREAMBLE_CHIPS) if len(chip_buf) >= 64 else -1
            if pos >= 0:
                align_offset = pos % 32
                aligned = chip_buf[align_offset:]
                syms = chips_to_symbols(aligned)
                if syms:
                    bits = symbols_to_bits(syms)
                    data = bits_to_bytes_lsb(bits)
                    frame, pos = find_preamble(data)
                    if frame is not None:
                        last_clear = time.time()
                        payload_len = frame[5] - 2
                        if payload_len > 0 and len(frame) >= 6 + payload_len + 2:
                            mac = frame[6:6+payload_len]
                            fcs_rx = frame[6+payload_len] | (frame[6+payload_len+1] << 8)
                            fcs_calc = crc16_ccitt(mac)
                            fcs_ok = (fcs_rx == fcs_calc)
                        else:
                            fcs_ok = False

                        if fcs_ok:
                            crc_ok_packets += 1
                        else:
                            preamble_only_packets += 1

                        chip_pos = pos * 32
                        print(f"\n=== PREAMBLE at byte {pos}  FCS={'OK' if fcs_ok else 'FAIL'} "
                              f"crc_ok:{crc_ok_packets} preamble_only:{preamble_only_packets} ===")
                        print(f"Chips around preamble: {chip_buf[chip_pos:chip_pos+64]}")
                        print(f"Symbol distances: {[d for _,d in syms[pos*2:pos*2+8]]}")
                        if fcs_ok:
                            print(f"Payload: {[hex(b) for b in mac]}")
                        print(f"Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")
                        chip_buf = ""

        if time.time() - last_report >= 2.0 and zmq_msgs > 0:
            preview = chip_buf[:100] if chip_buf else "(empty)"
            print(f"[msgs:{zmq_msgs} chips:{len(chip_buf)} "
                  f"crc_ok:{crc_ok_packets} preamble_only:{preamble_only_packets} "
                  f"raw:{preview}]")
            last_report = time.time()

except KeyboardInterrupt:
    zmq_sub.close(); gr_block_obj.stop(); gr_block_obj.wait()
    print("exit")
