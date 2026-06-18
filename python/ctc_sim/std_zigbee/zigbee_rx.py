#!/usr/bin/env python3
# coding=utf-8
import argparse, os, sys, time, zmq, signal, numpy as np
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
PREAMBLE_MAX_DIST = 12
PREAMBLE_BYTES = 4
SFD = 0xA7
MAX_PHR_LEN = 127
MIN_FRAME_BYTES = PREAMBLE_BYTES + 2
MIN_FRAME_SYMBOLS = MIN_FRAME_BYTES * 2
PREAMBLE_ONLY_BYTES = [0x00] * PREAMBLE_BYTES + [SFD, 0x00]
PREAMBLE_ONLY_SYMBOLS = [0, 0, 0, 0, 0, 0, 0, 0, 14, 5, 0, 0]
PREAMBLE_ONLY_MAX_SYMBOL_DIST = 144
PHASE_TEMPLATE_MAX_DIST = 110
PHASE_DETECT_CONFIRMATIONS = 2
BLUEBEE_SCAN_CHIPS = 2048
BLUEBEE_SCAN_PERIOD = 0.5
PHASE_SCAN_PERIOD = 0.05
PHASE_SCAN_CHIPS = 4096
PHASE_MAX_CHIPS = 9600
DIAG_SCAN_CHIPS = 4096
BYTE_TO_CHIPS = tuple(
    "".join("1" if (value >> bit) & 1 else "0" for bit in range(8))
    for value in range(256)
)


def chips_to_int(chips):
    value = 0
    for ch in chips:
        value = (value << 1) | (1 if ch == "1" else 0)
    return value


def bluebee_constrain_pair(pair_bits):
    return "11" if pair_bits.count("1") > 1 else "00"


def bluebee_legacy_symbol_chips(symbol):
    chips = CHIP_MAP[symbol]
    return "".join(bluebee_constrain_pair(chips[i:i+2]) for i in range(0, 32, 2))


def bluebee_candidate_chips(chips):
    candidates = [""]
    for i in range(0, 32, 2):
        pair = chips[i:i+2]
        choices = (pair,) if pair in ("00", "11") else ("00", "11")
        candidates = [prefix + choice for prefix in candidates for choice in choices]
    return candidates


def bluebee_optimized_symbol_chips(symbol):
    chips = CHIP_MAP[symbol]
    best = None
    for candidate in bluebee_candidate_chips(chips):
        intra = sum(1 for a, b in zip(candidate, chips) if a != b)
        inter = [
            sum(1 for a, b in zip(candidate, other) if a != b)
            for other_symbol, other in enumerate(CHIP_MAP)
            if other_symbol != symbol
        ]
        score = (-intra, min(inter), sum(inter), candidate)
        if best is None or score > best[0]:
            best = (score, candidate)
    return best[1]


def build_phase_templates():
    templates = []
    for name, builder in (("optimized", bluebee_optimized_symbol_chips), ("legacy", bluebee_legacy_symbol_chips)):
        chips = "".join(builder(symbol) for symbol in PREAMBLE_ONLY_SYMBOLS)
        templates.append({
            "name": name,
            "chips": chips,
            "int": chips_to_int(chips),
            "len": len(chips),
            "mask": (1 << len(chips)) - 1,
        })
    return templates


PHASE_TEMPLATES = build_phase_templates()

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
    return "".join(BYTE_TO_CHIPS[byte] for byte in data)

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

def invert_chips(chips):
    return chips.translate(str.maketrans("01", "10"))

def swap_chip_pairs(chips):
    usable = len(chips) - (len(chips) % 2)
    swapped = []
    for i in range(0, usable, 2):
        swapped.append(chips[i+1])
        swapped.append(chips[i])
    if usable < len(chips):
        swapped.append(chips[-1])
    return "".join(swapped)

def chip_variants(chips):
    variants = [("normal", chips), ("inverted", invert_chips(chips))]
    swapped = swap_chip_pairs(chips)
    variants.append(("pair_swap", swapped))
    variants.append(("inverted_pair_swap", invert_chips(swapped)))
    return variants

def symbol_window_distance(syms, start, target):
    if start + len(target) > len(syms):
        return None
    mismatch = sum(1 for i in range(len(target)) if syms[start+i][0] != target[i])
    dist = sum(syms[start+i][1] for i in range(len(target)))
    return mismatch, dist


def chips_to_bytes_preview(chips, max_bytes=12):
    data = []
    usable = min(len(chips) - len(chips) % 8, max_bytes * 8)
    for i in range(0, usable, 8):
        value = 0
        for bit_idx, ch in enumerate(chips[i:i+8]):
            if ch == "1":
                value |= 1 << bit_idx
        data.append(value)
    return data


def chip_stats(chips):
    if not chips:
        return 0.0, 0.0
    ones = chips.count("1") / len(chips)
    transitions = 0
    if len(chips) > 1:
        transitions = sum(1 for a, b in zip(chips, chips[1:]) if a != b) / (len(chips) - 1)
    return ones, transitions


def closest_exact_preamble(chips):
    if len(chips) < 32:
        return None
    best = None
    for pos in range(0, len(chips) - 31):
        dist = sum(1 for a, b in zip(chips[pos:pos+32], PREAMBLE_CHIPS) if a != b)
        if best is None or dist < best[0]:
            best = (dist, pos)
    return best


def best_preamble_only_diagnostic(chips):
    best = None
    for variant_name, variant_chips in chip_variants(chips):
        for align_offset in range(32):
            syms = chips_to_symbols(variant_chips[align_offset:])
            if len(syms) < len(PREAMBLE_ONLY_SYMBOLS):
                continue
            for sym_pos in range(0, len(syms) - len(PREAMBLE_ONLY_SYMBOLS) + 1):
                mismatch, dist = symbol_window_distance(syms, sym_pos, PREAMBLE_ONLY_SYMBOLS)
                score = mismatch * 1000 + dist
                if best is None or score < best["score"]:
                    chip_pos = align_offset + sym_pos * 32
                    window = syms[sym_pos:sym_pos+len(PREAMBLE_ONLY_SYMBOLS)]
                    best = {
                        "score": score,
                        "chip_pos": chip_pos,
                        "align": align_offset,
                        "sym_pos": sym_pos,
                        "variant": variant_name,
                        "mismatch": mismatch,
                        "dist": dist,
                        "symbols": [v for v, _ in window],
                        "distances": [d for _, d in window],
                        "chips": variant_chips[chip_pos:chip_pos+96],
                        "bytes": chips_to_bytes_preview(variant_chips[chip_pos:chip_pos+384]),
                    }
    return best


def print_diag_report(chip_buf, raw_msgs):
    scan = chip_buf[-DIAG_SCAN_CHIPS:]
    ones, transitions = chip_stats(scan)
    raw_lens = [len(msg) for msg in raw_msgs]
    exact = closest_exact_preamble(scan)
    best = best_preamble_only_diagnostic(scan)
    lens_text = "none" if not raw_lens else f"count={len(raw_lens)} min={min(raw_lens)} max={max(raw_lens)}"
    exact_text = "none" if exact is None else f"dist={exact[0]} pos={exact[1]}"
    print(f"[diag] scan_chips={len(scan)} ones={ones:.3f} transitions={transitions:.3f} raw_msgs={lens_text} exact_sym0={exact_text}")
    if best is None:
        print("[diag] preamble_window=none")
        return
    print(
        f"[diag] best_preamble variant={best['variant']} align={best['align']} "
        f"chip={best['chip_pos']} mismatch={best['mismatch']} dist={best['dist']} score={best['score']}"
    )
    print(f"[diag] symbols={best['symbols']} distances={best['distances']}")
    print(f"[diag] bytes={' '.join(f'{b:02X}' for b in best['bytes'])}")
    print(f"[diag] chips={best['chips']}")

def read_complex64_tail(path, max_samples):
    try:
        size = os.path.getsize(path)
    except OSError:
        return np.array([], dtype=np.complex64)
    sample_size = np.dtype(np.complex64).itemsize
    usable = size - (size % sample_size)
    if usable <= 0:
        return np.array([], dtype=np.complex64)
    read_samples = min(max_samples, usable // sample_size)
    with open(path, "rb") as f:
        f.seek(usable - read_samples * sample_size)
        return np.fromfile(f, dtype=np.complex64, count=read_samples)


def phase_chips_from_iq(iq, sample_rate, chip_rate, phase_offset, polarity, max_chips):
    if len(iq) < 2:
        return ""
    phase = np.angle(iq[1:] * np.conj(iq[:-1]))
    samples_per_chip = sample_rate / chip_rate
    chip_count = min(max_chips, int((len(phase) - phase_offset) / samples_per_chip))
    if chip_count <= 0:
        return ""
    sample_idx = np.rint(phase_offset + np.arange(chip_count) * samples_per_chip).astype(np.int64)
    sample_idx = sample_idx[sample_idx < len(phase)]
    signs = phase[sample_idx] >= 0
    if polarity == "inverted":
        signs = np.logical_not(signs)
    return "".join("1" if bit else "0" for bit in signs)


def best_phase_preamble_diagnostic(iq, sample_rate, chip_rate, max_chips):
    if len(iq) < 2:
        return None
    samples_per_chip = sample_rate / chip_rate
    phase_offsets = np.linspace(0, samples_per_chip, max(2, int(np.ceil(samples_per_chip)) * 4), endpoint=False)
    best = None
    for polarity in ("normal", "inverted"):
        for phase_offset in phase_offsets:
            chips = phase_chips_from_iq(iq, sample_rate, chip_rate, phase_offset, polarity, max_chips)
            for chip_align in range(32):
                syms = chips_to_symbols(chips[chip_align:])
                if len(syms) < len(PREAMBLE_ONLY_SYMBOLS):
                    continue
                for sym_pos in range(0, len(syms) - len(PREAMBLE_ONLY_SYMBOLS) + 1):
                    mismatch, dist = symbol_window_distance(syms, sym_pos, PREAMBLE_ONLY_SYMBOLS)
                    score = mismatch * 1000 + dist
                    if best is None or score < best["score"]:
                        chip_pos = chip_align + sym_pos * 32
                        window = syms[sym_pos:sym_pos+len(PREAMBLE_ONLY_SYMBOLS)]
                        best = {
                            "score": score,
                            "polarity": polarity,
                            "phase_offset": phase_offset,
                            "chip_align": chip_align,
                            "chip_pos": chip_pos,
                            "mismatch": mismatch,
                            "dist": dist,
                            "symbols": [v for v, _ in window],
                            "distances": [d for _, d in window],
                            "chips": chips[chip_pos:chip_pos+96],
                        }
    return best


def print_phase_diag_report(path, sample_rate, chip_rate, max_samples, max_chips):
    iq = read_complex64_tail(path, max_samples)
    if len(iq) == 0:
        print(f"[phase_diag] iq_samples=0 path={path}")
        return None
    power = float(np.mean(np.abs(iq) ** 2))
    peak = float(np.max(np.abs(iq)))
    best = best_phase_preamble_diagnostic(iq, sample_rate, chip_rate, max_chips)
    if best is None:
        print(f"[phase_diag] iq_samples={len(iq)} power={power:.4g} peak={peak:.4g} preamble_window=none")
        return None
    best["power"] = power
    best["peak"] = peak
    best["iq_samples"] = len(iq)
    print(
        f"[phase_diag] iq_samples={len(iq)} power={power:.4g} peak={peak:.4g} "
        f"polarity={best['polarity']} phase_offset={best['phase_offset']:.3f} "
        f"align={best['chip_align']} chip={best['chip_pos']} "
        f"mismatch={best['mismatch']} dist={best['dist']} score={best['score']}"
    )
    print(f"[phase_diag] symbols={best['symbols']} distances={best['distances']}")
    print(f"[phase_diag] chips={best['chips']}")
    return best

def find_preamble_only_window(chips):
    best = None
    for variant_name, variant_chips in chip_variants(chips):
        for align_offset in range(32):
            syms = chips_to_symbols(variant_chips[align_offset:])
            if len(syms) < len(PREAMBLE_ONLY_SYMBOLS):
                continue
            for sym_pos in range(0, len(syms) - len(PREAMBLE_ONLY_SYMBOLS) + 1):
                mismatch, dist = symbol_window_distance(syms, sym_pos, PREAMBLE_ONLY_SYMBOLS)
                score = mismatch * 1000 + dist
                if best is None or score < best[0]:
                    chip_pos = align_offset + sym_pos * 32
                    best = (score, chip_pos, syms, sym_pos, variant_name, mismatch, dist)
                if mismatch == 0 and dist <= PREAMBLE_ONLY_MAX_SYMBOL_DIST:
                    chip_pos = align_offset + sym_pos * 32
                    return PREAMBLE_ONLY_BYTES, chip_pos, syms, sym_pos, variant_name, dist
    if best is None:
        return None, -1, None, 0, "none", None
    return None, best[1], best[2], best[3], best[4], f"mismatch:{best[5]} dist:{best[6]}"

def find_phase_preamble_window(chips, max_mismatch=0, max_dist=PHASE_TEMPLATE_MAX_DIST):
    if len(chips) < PHASE_TEMPLATES[0]["len"]:
        return None, -1, None, 0, None, None

    best = None
    window_len = PHASE_TEMPLATES[0]["len"]
    window = chips_to_int(chips[:window_len])

    for pos in range(0, len(chips) - window_len + 1):
        if pos > 0:
            window = ((window << 1) & PHASE_TEMPLATES[0]["mask"]) | (1 if chips[pos + window_len - 1] == "1" else 0)

        for template in PHASE_TEMPLATES:
            dist = (window ^ template["int"]).bit_count()
            if best is None or dist < best["dist"]:
                best = {
                    "score": dist,
                    "chip_pos": pos,
                    "align": pos % 32,
                    "sym_pos": 0,
                    "mismatch": 0 if dist <= max_dist else 1,
                    "dist": dist,
                    "template": template["name"],
                    "chips": chips[pos:pos+96],
                }
            if dist <= max_dist:
                syms = chips_to_symbols(chips[pos:pos+window_len])
                best.update({
                    "symbols": [v for v, _ in syms[:len(PREAMBLE_ONLY_SYMBOLS)]],
                    "distances": [d for _, d in syms[:len(PREAMBLE_ONLY_SYMBOLS)]],
                })
                return PREAMBLE_ONLY_BYTES, pos, syms, 0, dist, best

    if best is not None:
        syms = chips_to_symbols(chips[best["chip_pos"]:best["chip_pos"]+window_len])
        best.update({
            "symbols": [v for v, _ in syms[:len(PREAMBLE_ONLY_SYMBOLS)]],
            "distances": [d for _, d in syms[:len(PREAMBLE_ONLY_SYMBOLS)]],
        })
    return None, -1, None, 0, None, best


def find_preamble(data):
    preamble = [0x00] * PREAMBLE_BYTES
    for i in range(0, len(data) - MIN_FRAME_BYTES + 1):
        if data[i:i+PREAMBLE_BYTES] != preamble or data[i+PREAMBLE_BYTES] != SFD:
            continue
        phr_len = data[i+PREAMBLE_BYTES+1]
        total_len = PREAMBLE_BYTES + 2 + phr_len
        if phr_len > MAX_PHR_LEN:
            continue
        if i + total_len <= len(data):
            return data[i:i+total_len], i
    return None, -1

def preamble_candidates(chips, max_dist=PREAMBLE_MAX_DIST):
    usable = len(chips) - 32 + 1
    for pos in range(0, usable):
        chunk = chips[pos:pos+32]
        dist = sum(1 for a, b in zip(chunk, PREAMBLE_CHIPS) if a != b)
        if dist <= max_dist:
            yield pos, dist


def find_frame_window(chips):
    search_pos = 0
    while True:
        candidate = chips.find(PREAMBLE_CHIPS, search_pos)
        if candidate < 0:
            return None, -1, None, 0

        align_offset = candidate % 32
        candidate_symbol = (candidate - align_offset) // 32
        start_symbol = max(candidate_symbol - PREAMBLE_BYTES * 2, 0)
        start_chip = align_offset + start_symbol * 32
        syms = chips_to_symbols(chips[start_chip:])
        if syms:
            bits = symbols_to_bits(syms)
            data = bits_to_bytes_lsb(bits)
            frame, byte_pos = find_preamble(data)
            if frame is not None:
                local_symbol = byte_pos * 2
                preamble_chip = start_chip + local_symbol * 32
                return frame, preamble_chip, syms, local_symbol

        search_pos = candidate + 1


parser = argparse.ArgumentParser(description="Receive ZigBee/BlueBee chips from HackRF")
parser.add_argument("--channel", type=int, default=26, help="ZigBee channel, use 26 for BlueBee BLE ch39 or 11 for normal ZigBee ch11")
parser.add_argument("--no-bluebee-scan", action="store_true", help="disable BlueBee short preamble/SFD side scan")
parser.add_argument("--no-phase-rx", action="store_true", help="disable fast BlueBee phase-difference ZMQ detector")
parser.add_argument("--phase-zmq", default="tcp://127.0.0.1:55557", help="ZMQ endpoint for phase-difference BlueBee chips")
parser.add_argument("--phase-keep-offset", type=int, default=0, help="sample offset 0..4 used by the GNU Radio phase chip sampler")
parser.add_argument("--phase-scan-period", type=float, default=PHASE_SCAN_PERIOD, help="seconds between fast phase-domain BlueBee scans")
parser.add_argument("--phase-hit-print-every", type=int, default=1, help="print every N phase preamble hits; use larger values to reduce stdout overhead")
parser.add_argument("--phase-detect-confirmations", type=int, default=PHASE_DETECT_CONFIRMATIONS, help="consecutive phase-template matches required before counting a BlueBee preamble")
parser.add_argument("--diag", action="store_true", help="print periodic nearest-preamble diagnostics even when no packet is detected")
parser.add_argument("--diag-period", type=float, default=1.0, help="seconds between --diag reports")
parser.add_argument("--iq-output", default=None, help="optional complex64 filtered-IQ recording path for bluebee_phase_analyze.py")
parser.add_argument("--phase-diag", action="store_true", help="periodically search the filtered IQ tail using paper-style phase-difference slicing")
parser.add_argument("--phase-diag-period", type=float, default=1.0, help="seconds between --phase-diag reports")
parser.add_argument("--phase-diag-samples", type=int, default=50000, help="complex64 IQ tail samples used by each --phase-diag report")
parser.add_argument("--phase-diag-chips", type=int, default=4096, help="maximum sliced chips searched by each --phase-diag report")
parser.add_argument("--phase-detect-max-mismatch", type=int, default=0, help="maximum symbol mismatch accepted as a phase-domain BlueBee preamble hit")
parser.add_argument("--phase-detect-max-dist", type=int, default=PHASE_TEMPLATE_MAX_DIST, help="maximum template Hamming distance accepted as a phase-domain BlueBee preamble hit")
parser.add_argument("--freq-offset", type=float, default=0.0, help="optional RX frequency offset in Hz applied in gr_zigbee")
args = parser.parse_args()
if args.phase_diag and not args.iq_output:
    args.iq_output = "/tmp/zigbee_rx_phase_diag.c64"
if args.iq_output:
    open(args.iq_output, "wb").close()

gr_block_obj = gr_block()
gr_block_obj.set_zigbee_channel(args.channel)
if hasattr(gr_block_obj, "set_phase_keep_offset"):
    gr_block_obj.set_phase_keep_offset(args.phase_keep_offset)
if args.freq_offset:
    gr_block_obj.set_freq_offset(args.freq_offset)
if args.iq_output:
    gr_block_obj.set_iq_output(args.iq_output)
gr_block_obj.start()
print(f"RX: ch{args.channel} {gr_block_obj.get_freq()/1e6:.1f} MHz "
      f"sr={gr_block_obj.get_sample_rate()/1e6:.1f} MHz "
      f"bluebee_scan={'off' if args.no_bluebee_scan else 'on'} "
      f"diag={'on' if args.diag else 'off'} iq={args.iq_output or 'off'} freq_offset={args.freq_offset:g}")

zmq_sub = ZMQSubscriber()
phase_zmq_sub = None if args.no_phase_rx else ZMQSubscriber(addr=args.phase_zmq)
zmq_msgs = 0
phase_zmq_msgs = 0
crc_ok_packets = 0
preamble_only_packets = 0
phase_preamble_hits = 0
phase_candidate_streak = 0
phase_best_dist = None
last_report = time.time()
last_clear = time.time()
last_bluebee_scan = 0.0
last_phase_scan = 0.0
last_diag = 0.0
last_phase_diag = 0.0
last_raw_msgs = []
last_phase_raw_msgs = []
last_rx_time = time.time()
chip_buf = ""
phase_chip_buf = ""
MAX_CHIPS = 9600

try:
    while True:
        raw_msgs = zmq_sub.read_available()
        phase_raw_msgs = phase_zmq_sub.read_available() if phase_zmq_sub is not None else []
        if raw_msgs:
            last_raw_msgs = raw_msgs
            last_rx_time = time.time()
            zmq_msgs += len(raw_msgs)
            chips = "".join(unpack_bytes_to_chips(raw) for raw in raw_msgs if raw)
            chip_buf += chips
            if len(chip_buf) > MAX_CHIPS:
                chip_buf = chip_buf[-MAX_CHIPS:]

        if phase_raw_msgs:
            last_phase_raw_msgs = phase_raw_msgs
            last_rx_time = time.time()
            phase_zmq_msgs += len(phase_raw_msgs)
            phase_chips = "".join(unpack_bytes_to_chips(raw) for raw in phase_raw_msgs if raw)
            phase_chip_buf += phase_chips
            if len(phase_chip_buf) > PHASE_MAX_CHIPS:
                phase_chip_buf = phase_chip_buf[-PHASE_MAX_CHIPS:]

        # Clear stale noise buffer only after the ZMQ stream has gone idle.
        if chip_buf and time.time() - last_rx_time > 3.0:
            chip_buf = ""
            phase_chip_buf = ""
            last_clear = time.time()

        if (not args.no_phase_rx
                and len(phase_chip_buf) >= MIN_FRAME_SYMBOLS * 32
                and time.time() - last_phase_scan >= args.phase_scan_period):
            last_phase_scan = time.time()
            phase_scan_buf = phase_chip_buf[-PHASE_SCAN_CHIPS:]
            phase_frame, phase_local_chip_pos, phase_syms, phase_sym_pos, phase_dist, phase_best = find_phase_preamble_window(
                phase_scan_buf,
                max_mismatch=args.phase_detect_max_mismatch,
                max_dist=args.phase_detect_max_dist,
            )
            phase_best_dist = phase_best["dist"] if phase_best is not None else None
            if phase_frame is None:
                phase_candidate_streak = 0
            else:
                phase_candidate_streak += 1
                phase_chip_pos = len(phase_chip_buf) - len(phase_scan_buf) + phase_local_chip_pos
                if phase_candidate_streak >= args.phase_detect_confirmations:
                    preamble_only_packets += 1
                    phase_preamble_hits += 1
                    phase_candidate_streak = 0
                    if args.phase_hit_print_every > 0 and phase_preamble_hits % args.phase_hit_print_every == 0:
                        print(
                            f"\n=== PHASE PREAMBLE at chip {phase_chip_pos} "
                            f"hit:{phase_preamble_hits} crc_ok:{crc_ok_packets} "
                            f"preamble_only:{preamble_only_packets} dist:{phase_dist} "
                            f"align:{phase_local_chip_pos % 32} template:{phase_best.get('template', 'symbol')} ==="
                        )
                        print(f"Phase symbols: {[v for v,_ in phase_syms[phase_sym_pos:phase_sym_pos+12]]}")
                        print(f"Phase distances: {[d for _,d in phase_syms[phase_sym_pos:phase_sym_pos+12]]}")
                        print(f"Phase chips: {phase_chip_buf[phase_chip_pos:phase_chip_pos+96]}")
                    phase_chip_buf = ""

        if len(chip_buf) >= MIN_FRAME_SYMBOLS * 32:
            frame, chip_pos, syms, sym_pos = find_frame_window(chip_buf)
            preamble_variant = "full"
            preamble_dist = None

            if frame is None and not args.no_bluebee_scan and time.time() - last_bluebee_scan >= BLUEBEE_SCAN_PERIOD:
                last_bluebee_scan = time.time()
                scan_buf = chip_buf[-BLUEBEE_SCAN_CHIPS:]
                frame, local_chip_pos, syms, sym_pos, preamble_variant, preamble_dist = find_preamble_only_window(scan_buf)
                if frame is not None:
                    chip_pos = len(chip_buf) - len(scan_buf) + local_chip_pos

            if frame is None:
                if args.diag and time.time() - last_diag >= args.diag_period:
                    last_diag = time.time()
                    print_diag_report(chip_buf, last_raw_msgs)
            else:
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

                print(f"\n=== PREAMBLE at chip {chip_pos}  FCS={'OK' if fcs_ok else 'FAIL'} "
                      f"crc_ok:{crc_ok_packets} preamble_only:{preamble_only_packets} "
                      f"variant:{preamble_variant} dist:{preamble_dist} ===")
                print(f"Chips around preamble: {chip_buf[chip_pos:chip_pos+64]}")
                print(f"Symbol distances: {[d for _,d in syms[sym_pos:sym_pos+12]]}")
                print(f"Symbols: {[v for v,_ in syms[sym_pos:sym_pos+12]]}")
                if fcs_ok:
                    print(f"Payload: {[hex(b) for b in mac]}")
                print(f"Frame bytes: {' '.join(f'{b:02X}' for b in frame)}")
                chip_buf = ""

        if args.phase_diag and time.time() - last_phase_diag >= args.phase_diag_period:
            last_phase_diag = time.time()
            phase_best = print_phase_diag_report(
                args.iq_output,
                gr_block_obj.get_sample_rate(),
                2e6,
                args.phase_diag_samples,
                args.phase_diag_chips,
            )
            if (phase_best is not None
                    and phase_best["mismatch"] <= args.phase_detect_max_mismatch
                    and phase_best["dist"] <= args.phase_detect_max_dist):
                phase_preamble_hits += 1
                print(
                    f"=== PHASE PREAMBLE hit:{phase_preamble_hits} "
                    f"mismatch:{phase_best['mismatch']} dist:{phase_best['dist']} "
                    f"polarity:{phase_best['polarity']} offset:{phase_best['phase_offset']:.3f} "
                    f"align:{phase_best['chip_align']} ==="
                )

        if time.time() - last_report >= 2.0 and zmq_msgs > 0:
            preview = chip_buf[:100] if chip_buf else "(empty)"
            ones, transitions = chip_stats(chip_buf[-DIAG_SCAN_CHIPS:])
            phase_ones, phase_transitions = chip_stats(phase_chip_buf[-DIAG_SCAN_CHIPS:])
            best_text = "none" if phase_best_dist is None else str(phase_best_dist)
            print(f"[msgs:{zmq_msgs} phase_msgs:{phase_zmq_msgs} chips:{len(chip_buf)} phase_chips:{len(phase_chip_buf)} "
                  f"crc_ok:{crc_ok_packets} preamble_only:{preamble_only_packets} "
                  f"phase_preamble:{phase_preamble_hits} phase_best_dist:{best_text} "
                  f"phase_streak:{phase_candidate_streak} "
                  f"ones:{ones:.3f} trans:{transitions:.3f} "
                  f"phase_ones:{phase_ones:.3f} phase_trans:{phase_transitions:.3f} raw:{preview}]")
            last_report = time.time()

except KeyboardInterrupt:
    zmq_sub.close()
    if phase_zmq_sub is not None:
        phase_zmq_sub.close()
    gr_block_obj.stop(); gr_block_obj.wait()
    print("exit")
