#!/usr/bin/env python3
# coding=utf-8

import math
import matplotlib.pyplot as plt


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


def read_iq(path):
    i_list = []
    q_list = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            i_list.append(float(parts[0]))
            q_list.append(float(parts[1]))
    if not i_list:
        raise ValueError("no samples in zigbee_iq.txt")
    return i_list, q_list


def half_sine_pulse(samples_per_chip):
    return [math.sin(math.pi * (n + 0.5) / samples_per_chip) for n in range(samples_per_chip)]


def despread(i_list, q_list, samples_per_chip=8):
    pulse = half_sine_pulse(samples_per_chip)
    delay = samples_per_chip // 2
    if len(q_list) > delay:
        q_aligned = q_list[delay:]
    else:
        q_aligned = []
    if len(q_aligned) < len(i_list):
        q_aligned.extend([0.0] * (len(i_list) - len(q_aligned)))

    chip_count = len(i_list) // samples_per_chip
    i_chips = []
    q_chips = []
    for k in range(chip_count):
        s = k * samples_per_chip
        e = s + samples_per_chip
        i_val = sum(i_list[s:e][n] * pulse[n] for n in range(samples_per_chip))
        q_val = sum(q_aligned[s:e][n] * pulse[n] for n in range(samples_per_chip))
        i_chips.append(1 if i_val >= 0 else 0)
        q_chips.append(1 if q_val >= 0 else 0)

    chips = []
    for i, q in zip(i_chips, q_chips):
        chips.append(str(i))
        chips.append(str(q))
    return "".join(chips)


def chips_to_symbols(chips):
    symbols = []
    usable = (len(chips) // 32) * 32
    for i in range(0, usable, 32):
        chunk = chips[i : i + 32]
        best_symbol = 0
        best_dist = 33
        for s, ref in enumerate(CHIP_MAP):
            dist = sum(1 for a, b in zip(chunk, ref) if a != b)
            if dist < best_dist:
                best_dist = dist
                best_symbol = s
        symbols.append(best_symbol)
    return symbols


def symbols_to_bits(symbols):
    return "".join(f"{s:04b}" for s in symbols)


def plot_iq(i_list, q_list, samples=2000):
    n = min(samples, len(i_list))
    t = list(range(n))
    plt.figure(figsize=(10, 5))
    plt.plot(t, i_list[:n], label="I")
    plt.plot(t, q_list[:n], label="Q")
    plt.title("ZigBee IQ (time domain)")
    plt.xlabel("Sample")
    plt.ylabel("Amplitude")
    plt.legend()


def plot_constellation(i_list, q_list, samples_per_chip=8, samples=2000):
    delay = samples_per_chip // 2
    if len(q_list) > delay:
        q_aligned = q_list[delay:]
    else:
        q_aligned = []
    if len(q_aligned) < len(i_list):
        q_aligned.extend([0.0] * (len(i_list) - len(q_aligned)))
    n = min(samples, len(i_list))
    plt.figure(figsize=(5, 5))
    plt.scatter(i_list[:n], q_aligned[:n], s=4, alpha=0.6)
    plt.title("ZigBee IQ (constellation)")
    plt.xlabel("I")
    plt.ylabel("Q")
    plt.axis("equal")


def main():
    i_list, q_list = read_iq("zigbee_iq.txt")
    plot_iq(i_list, q_list)
    plot_constellation(i_list, q_list)

    chips = despread(i_list, q_list, samples_per_chip=8)
    symbols = chips_to_symbols(chips)
    bits = symbols_to_bits(symbols)
    print(f"samples: {len(i_list)}")
    print(f"chips: {len(chips)}")
    print(f"symbols: {len(symbols)}")
    print(f"bits: {len(bits)}")
    print("decoded bits (first 128):")
    print(bits[:128])

    plt.show()


if __name__ == "__main__":
    main()
