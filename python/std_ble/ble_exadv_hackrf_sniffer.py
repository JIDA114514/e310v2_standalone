#!/usr/bin/env python3
# coding=utf-8

import argparse
import signal
import sys
import time

import zmq

from bsp_algorithm import bsp_algorithm


BLE_AA_AND_PREAMBLE = bytes([0xAA, 0xD6, 0xBE, 0x89, 0x8E])
BLE_PDU_TYPES = {
    0x00: "ADV_IND",
    0x01: "ADV_DIRECT_IND",
    0x02: "ADV_NONCONN_IND",
    0x03: "SCAN_REQ/AUX_SCAN_REQ",
    0x04: "SCAN_RSP/AUX_SCAN_RSP",
    0x05: "CONNECT_IND/AUX_CONNECT_REQ",
    0x06: "ADV_SCAN_IND",
    0x07: "ADV_EXT_IND/AUX_ADV_IND",
}


def ble_channel_freq_hz(channel):
    if channel == 37:
        return 2402000000
    if channel == 38:
        return 2426000000
    if channel == 39:
        return 2480000000
    if 0 <= channel <= 10:
        return int((2404 + 2 * channel) * 1e6)
    if 11 <= channel <= 36:
        return int((2406 + 2 * channel) * 1e6)
    raise ValueError(f"unsupported BLE channel {channel}")


def fmt_hex(data):
    return " ".join(f"{x:02X}" for x in data)


def mac_from_adv_addr(payload):
    if len(payload) < 6:
        return None
    return ":".join(f"{x:02X}" for x in reversed(payload[:6]))


def decode_adi(data):
    if len(data) < 2:
        return None
    value = data[0] | (data[1] << 8)
    return (value >> 12) & 0x0F, value & 0x0FFF


def decode_aux_ptr(data):
    if len(data) < 3:
        return None
    channel = data[0] & 0x3F
    ca = (data[0] >> 6) & 0x01
    offset_units = (data[0] >> 7) & 0x01
    aux_offset = data[1] | ((data[2] & 0x1F) << 8)
    phy = (data[2] >> 5) & 0x07
    offset_us = aux_offset * (300 if offset_units else 30)
    return {
        "channel": channel,
        "ca": ca,
        "offset_units": offset_units,
        "offset_us": offset_us,
        "phy": phy,
    }


def parse_ad_structures(adv_data):
    items = []
    pos = 0
    while pos < len(adv_data):
        length = adv_data[pos]
        if length == 0:
            break
        end = pos + 1 + length
        if end > len(adv_data):
            items.append(("malformed", adv_data[pos:]))
            break
        items.append((adv_data[pos + 1], adv_data[pos + 2:end]))
        pos = end
    return items


def describe_adv_data(adv_data):
    parts = []
    name = None
    flags = None
    for ad_type, value in parse_ad_structures(adv_data):
        if ad_type == "malformed":
            parts.append(f"malformed={fmt_hex(value)}")
        elif ad_type == 0x01:
            flags = value[0] if value else None
            parts.append(f"Flags=0x{flags:02X}" if flags is not None else "Flags=<empty>")
        elif ad_type in (0x08, 0x09):
            name = bytes(value).decode("ascii", errors="replace")
            parts.append(f"Name={name}")
        elif ad_type == 0xFF:
            company = value[0] | (value[1] << 8) if len(value) >= 2 else None
            if company is None:
                parts.append(f"Mfg={fmt_hex(value)}")
            else:
                parts.append(f"Mfg=0x{company:04X}:{fmt_hex(value[2:])}")
        else:
            parts.append(f"AD{ad_type:02X}={fmt_hex(value)}")
    return {
        "text": ", ".join(parts) if parts else "<none>",
        "name": name,
        "flags": flags,
    }


def parse_ext_adv_payload(payload):
    info = {
        "ext_hdr_len": 0,
        "adv_mode": None,
        "flags": 0,
        "adva": None,
        "adi": None,
        "aux_ptr": None,
        "adv_data": [],
        "adv_data_text": "<none>",
        "name": None,
    }
    if not payload:
        return info
    ext_hdr_len = payload[0] & 0x3F
    info["ext_hdr_len"] = ext_hdr_len
    info["adv_mode"] = (payload[0] >> 6) & 0x03
    if len(payload) < 1 + ext_hdr_len:
        info["short"] = True
        return info

    ext_hdr = payload[1:1 + ext_hdr_len]
    adv_data = payload[1 + ext_hdr_len:]
    info["adv_data"] = adv_data
    if not ext_hdr:
        parsed_ad = describe_adv_data(adv_data)
        info["adv_data_text"] = parsed_ad["text"]
        info["name"] = parsed_ad["name"]
        return info

    flags = ext_hdr[0]
    info["flags"] = flags
    pos = 1
    if flags & 0x01 and pos + 6 <= len(ext_hdr):
        info["adva"] = mac_from_adv_addr(ext_hdr[pos:pos + 6])
        pos += 6
    if flags & 0x02 and pos + 6 <= len(ext_hdr):
        pos += 6
    if flags & 0x04 and pos + 2 <= len(ext_hdr):
        info["adi"] = decode_adi(ext_hdr[pos:pos + 2])
        pos += 2
    if flags & 0x08 and pos + 18 <= len(ext_hdr):
        pos += 18
    if flags & 0x10 and pos + 3 <= len(ext_hdr):
        info["aux_ptr"] = decode_aux_ptr(ext_hdr[pos:pos + 3])
        pos += 3
    if flags & 0x20 and pos + 1 <= len(ext_hdr):
        pos += 1
    parsed_ad = describe_adv_data(adv_data)
    info["adv_data_text"] = parsed_ad["text"]
    info["name"] = parsed_ad["name"]
    return info


class BleStreamParser:
    def __init__(self, channel, keep_crc_errors=False):
        self.channel = channel
        self.keep_crc_errors = keep_crc_errors
        self.buffer = bytearray()
        self.stream_offset = 0
        self.detected = 0
        self.crc_errors = 0

    def insert_data(self, data):
        if isinstance(data, str):
            data = data.encode("latin-1")
        self.buffer.extend(data)
        if len(self.buffer) > 65536:
            drop = len(self.buffer) - 4096
            del self.buffer[:drop]
            self.stream_offset += drop

    def poll(self):
        packets = []
        while True:
            start = self.buffer.find(BLE_AA_AND_PREAMBLE)
            if start < 0:
                if len(self.buffer) > len(BLE_AA_AND_PREAMBLE):
                    drop = len(self.buffer) - len(BLE_AA_AND_PREAMBLE)
                    del self.buffer[:drop]
                    self.stream_offset += drop
                return packets
            if start > 0:
                del self.buffer[:start]
                self.stream_offset += start
                start = 0
            if len(self.buffer) < 7:
                return packets

            header = bsp_algorithm.bt_dewhitening(list(self.buffer[5:7]), self.channel)
            length = header[1] & 0x3F
            total = 5 + 2 + length + 3
            if len(self.buffer) < total:
                return packets

            pdu = bsp_algorithm.bt_dewhitening(list(self.buffer[5:5 + 2 + length + 3]), self.channel)
            crc = pdu[-3:]
            calc_crc = bsp_algorithm.bt_crc(pdu, 2 + length)
            crc_ok = crc == calc_crc
            if crc_ok or self.keep_crc_errors:
                self.detected += 1
                packet = self._build_packet(pdu, crc_ok, self.stream_offset + start, total)
                packets.append(packet)
            else:
                self.crc_errors += 1
            del self.buffer[:total]
            self.stream_offset += total

    def _build_packet(self, pdu, crc_ok, start, total):
        header0, header1 = pdu[0], pdu[1]
        pdu_type = header0 & 0x0F
        length = header1 & 0x3F
        payload = pdu[2:2 + length]
        ext = parse_ext_adv_payload(payload) if pdu_type == 0x07 else None
        kind = "other"
        if ext:
            flags = ext["flags"]
            if (flags & 0x11) == 0x11:
                kind = "primary"
            elif (flags & 0x04) and ext["adv_data"]:
                kind = "secondary"
            else:
                kind = "extended"
        return {
            "packet_no": self.detected,
            "channel": self.channel,
            "start": start,
            "total_bytes": total,
            "pdu": pdu,
            "pdu_type": pdu_type,
            "pdu_name": BLE_PDU_TYPES.get(pdu_type, f"UNKNOWN_{pdu_type}"),
            "length": length,
            "txadd": (header0 >> 6) & 0x01,
            "rxadd": (header0 >> 7) & 0x01,
            "crc_ok": crc_ok,
            "kind": kind,
            "ext": ext,
            "monotonic": time.monotonic(),
        }


class ExtendedAdvMatcher:
    def __init__(self, secondary_channel, match_window_s=1.0, expected_name=None):
        self.secondary_channel = secondary_channel
        self.match_window_s = match_window_s
        self.expected_name = expected_name
        self.primaries = []
        self.secondaries = []
        self.events = 0
        self.expired_primaries = 0
        self.expired_secondaries = 0

    @staticmethod
    def _adi_text(adi):
        return f"SID{adi[0]} DID{adi[1]}" if adi else "<none>"

    @staticmethod
    def _same_identity(primary, secondary):
        pri_ext = primary["ext"]
        sec_ext = secondary["ext"]
        aux = pri_ext["aux_ptr"]
        if not aux or aux["channel"] != secondary["channel"]:
            return False
        if pri_ext["adi"] and sec_ext["adi"] and pri_ext["adi"] != sec_ext["adi"]:
            return False
        if pri_ext["adva"] and sec_ext["adva"] and pri_ext["adva"] != sec_ext["adva"]:
            return False
        return True

    def _cleanup(self, now):
        lines = []
        keep = []
        for primary in self.primaries:
            if now - primary["monotonic"] <= self.match_window_s:
                keep.append(primary)
            else:
                self.expired_primaries += 1
                ext = primary["ext"]
                aux = ext["aux_ptr"]
                lines.append(
                    f"primary-only[{self.expired_primaries}] ch{primary['channel']} #{primary['packet_no']} "
                    f"ADI={self._adi_text(ext['adi'])} AdvA={ext['adva']} "
                    f"AuxPtr=ch{aux['channel']}+{aux['offset_us']}us"
                )
        self.primaries = keep

        keep = []
        for secondary in self.secondaries:
            if now - secondary["monotonic"] <= self.match_window_s:
                keep.append(secondary)
            else:
                self.expired_secondaries += 1
                ext = secondary["ext"]
                lines.append(
                    f"secondary-only[{self.expired_secondaries}] ch{secondary['channel']} #{secondary['packet_no']} "
                    f"ADI={self._adi_text(ext['adi'])} AdvA={ext['adva']} AdvData={ext['adv_data_text']}"
                )
        self.secondaries = keep
        return lines

    def _format_event(self, primary, secondary):
        self.events += 1
        pri_ext = primary["ext"]
        sec_ext = secondary["ext"]
        aux = pri_ext["aux_ptr"]
        wall_delta_ms = (secondary["monotonic"] - primary["monotonic"]) * 1000.0
        name_ok = ""
        if self.expected_name:
            name_ok = " name_ok=1" if sec_ext["name"] == self.expected_name else " name_ok=0"
        return (
            f"EXT_ADV_EVENT[{self.events}] primary_ch={primary['channel']} secondary_ch={secondary['channel']} "
            f"AuxPtr=ch{aux['channel']}+{aux['offset_us']}us phy{aux['phy']} "
            f"ADI={self._adi_text(sec_ext['adi'])} AdvA={sec_ext['adva']} "
            f"Name={sec_ext['name']} wall_delta_ms={wall_delta_ms:.3f}{name_ok}"
        )

    def _try_match_primary(self, primary):
        for idx in range(len(self.secondaries) - 1, -1, -1):
            secondary = self.secondaries[idx]
            if self._same_identity(primary, secondary):
                del self.secondaries[idx]
                return self._format_event(primary, secondary)
        return None

    def _try_match_secondary(self, secondary):
        for idx in range(len(self.primaries) - 1, -1, -1):
            primary = self.primaries[idx]
            if self._same_identity(primary, secondary):
                del self.primaries[idx]
                return self._format_event(primary, secondary)
        return None

    def feed(self, packet):
        lines = self._cleanup(packet["monotonic"])
        if packet["kind"] == "primary":
            line = self._try_match_primary(packet)
            if line:
                lines.append(line)
            else:
                self.primaries.append(packet)
        elif packet["kind"] == "secondary":
            line = self._try_match_secondary(packet)
            if line:
                lines.append(line)
            else:
                self.secondaries.append(packet)
        return lines

def describe_packet(packet):
    ext = packet["ext"]
    base = (
        f"[ch{packet['channel']} #{packet['packet_no']}] start={packet['start']} "
        f"type={packet['pdu_name']} len={packet['length']} crc={'OK' if packet['crc_ok'] else 'BAD'}"
    )
    if not ext:
        return base
    parts = [
        base,
        f"kind={packet['kind']}",
        f"ExtHdrLen={ext['ext_hdr_len']}",
        f"Flags=0x{ext['flags']:02X}",
    ]
    if ext["adva"]:
        parts.append(f"AdvA={ext['adva']}")
    if ext["adi"]:
        parts.append(f"ADI=SID{ext['adi'][0]} DID{ext['adi'][1]}")
    if ext["aux_ptr"]:
        aux = ext["aux_ptr"]
        parts.append(f"AuxPtr=ch{aux['channel']} off{aux['offset_us']}us phy{aux['phy']}")
    parts.append(f"AdvData={ext['adv_data_text']}")
    return " ".join(parts)


class ZmqSub:
    def __init__(self, context, address):
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

    def close(self):
        self.socket.close()


def build_top_block(args):
    from gnuradio import blocks
    from gnuradio import digital
    from gnuradio import filter
    from gnuradio import gr
    from gnuradio import zeromq as gr_zeromq
    from gnuradio.filter import firdes
    import numpy
    import osmosdr

    class DualBleTopBlock(gr.top_block):
        def __init__(self):
            gr.top_block.__init__(self, "BLE Extended Advertising Dual Channel Sniffer", catch_exceptions=True)

            sample_rate = float(args.sample_rate)
            data_rate = 1e6
            sps = int(sample_rate / data_rate)
            taps = firdes.low_pass(1, sample_rate, args.cutoff, args.transition)

            self.source = osmosdr.source(args=f"numchan=1 {args.osmosdr_args}")
            self.source.set_sample_rate(sample_rate)
            self.source.set_center_freq(args.center_freq, 0)
            self.source.set_freq_corr(0, 0)
            self.source.set_dc_offset_mode(0, 0)
            self.source.set_iq_balance_mode(0, 0)
            self.source.set_gain_mode(False, 0)
            self.source.set_gain(args.gain, 0)
            self.source.set_if_gain(args.if_gain, 0)
            self.source.set_bb_gain(args.bb_gain, 0)
            self.source.set_bandwidth(0, 0)
            self.head = blocks.head(gr.sizeof_gr_complex, int(args.duration * sample_rate)) if args.duration > 0 else None

            src = self.source
            if self.head:
                self.connect((self.source, 0), (self.head, 0))
                src = self.head

            self.blocks = []
            for channel, address in (
                (args.primary_channel, args.primary_zmq),
                (args.secondary_channel, args.secondary_zmq),
            ):
                offset = ble_channel_freq_hz(channel) - args.center_freq
                xlate = filter.freq_xlating_fir_filter_ccc(1, taps, offset, sample_rate)
                demod = digital.gfsk_demod(
                    samples_per_symbol=sps,
                    sensitivity=((numpy.pi * 0.5) / sps),
                    gain_mu=0.175,
                    mu=0.5,
                    omega_relative_limit=0.005,
                    freq_error=0.0,
                    verbose=False,
                    log=False,
                )
                pack = blocks.unpacked_to_packed_bb(1, gr.GR_LSB_FIRST)
                sink = gr_zeromq.pub_sink(gr.sizeof_char, 1, address, 100, False, -1, "", True, True)
                self.connect((src, 0), (xlate, 0))
                self.connect((xlate, 0), (demod, 0))
                self.connect((demod, 0), (pack, 0))
                self.connect((pack, 0), (sink, 0))
                self.blocks.extend([xlate, demod, pack, sink])

    return DualBleTopBlock()


def validate_args(args):
    primary_freq = ble_channel_freq_hz(args.primary_channel)
    secondary_freq = ble_channel_freq_hz(args.secondary_channel)
    if args.center_freq is None:
        args.center_freq = (primary_freq + secondary_freq) / 2.0
    max_offset = max(abs(primary_freq - args.center_freq), abs(secondary_freq - args.center_freq))
    if max_offset + args.cutoff + args.transition > args.sample_rate / 2.0:
        raise ValueError(
            "sample rate is too low for the requested channel spacing; "
            f"need > {2 * (max_offset + args.cutoff + args.transition):.0f} sps"
        )


def run(args):
    validate_args(args)
    tb = build_top_block(args)
    context = zmq.Context()
    primary_sub = ZmqSub(context, args.primary_zmq)
    secondary_sub = ZmqSub(context, args.secondary_zmq)
    poller = zmq.Poller()
    poller.register(primary_sub.socket, zmq.POLLIN)
    poller.register(secondary_sub.socket, zmq.POLLIN)

    parsers = {
        primary_sub.socket: BleStreamParser(args.primary_channel, keep_crc_errors=args.keep_crc_errors),
        secondary_sub.socket: BleStreamParser(args.secondary_channel, keep_crc_errors=args.keep_crc_errors),
    }
    matcher = ExtendedAdvMatcher(
        secondary_channel=args.secondary_channel,
        match_window_s=args.match_window_ms / 1000.0,
        expected_name=args.expected_name,
    )
    stop_at = time.monotonic() + args.duration if args.duration > 0 else None

    def stop_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        primary_sub.close()
        secondary_sub.close()
        context.term()
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print(
        "BLE exadv HackRF sniffer: "
        f"center={args.center_freq / 1e6:.3f}MHz sample_rate={args.sample_rate / 1e6:.1f}Msps "
        f"primary_ch={args.primary_channel} secondary_ch={args.secondary_channel}"
    )
    tb.start()
    try:
        while stop_at is None or time.monotonic() < stop_at:
            events = dict(poller.poll(50))
            for socket in (primary_sub.socket, secondary_sub.socket):
                if socket not in events:
                    continue
                data = socket.recv()
                parser = parsers[socket]
                parser.insert_data(data)
                for packet in parser.poll():
                    if args.print_packets:
                        print(describe_packet(packet))
                    for line in matcher.feed(packet):
                        print(line)
        print(
            "summary: "
            f"primary_packets={parsers[primary_sub.socket].detected} "
            f"secondary_packets={parsers[secondary_sub.socket].detected} "
            f"matched_events={matcher.events} "
            f"primary_crc_errors={parsers[primary_sub.socket].crc_errors} "
            f"secondary_crc_errors={parsers[secondary_sub.socket].crc_errors}"
        )
    finally:
        tb.stop()
        tb.wait()
        primary_sub.close()
        secondary_sub.close()
        context.term()


def main():
    parser = argparse.ArgumentParser(description="HackRF dual-channel BLE extended advertising sniffer")
    parser.add_argument("--primary-channel", type=int, default=37, help="primary BLE advertising channel")
    parser.add_argument("--secondary-channel", type=int, default=3, help="secondary BLE data channel from AuxPtr")
    parser.add_argument("--center-freq", type=float, default=None, help="HackRF center frequency in Hz; default midpoint")
    parser.add_argument("--sample-rate", type=float, default=20e6, help="HackRF sample rate")
    parser.add_argument("--duration", type=float, default=30.0, help="capture duration in seconds; <=0 runs until Ctrl-C")
    parser.add_argument("--cutoff", type=float, default=850e3, help="per-channel low-pass cutoff")
    parser.add_argument("--transition", type=float, default=300e3, help="per-channel low-pass transition width")
    parser.add_argument("--gain", type=float, default=40, help="HackRF RF gain")
    parser.add_argument("--if-gain", type=float, default=32, help="HackRF IF gain")
    parser.add_argument("--bb-gain", type=float, default=16, help="HackRF baseband gain")
    parser.add_argument("--osmosdr-args", default="hackrf=0", help="osmosdr source args")
    parser.add_argument("--primary-zmq", default="tcp://127.0.0.1:55557", help="internal ZMQ address for primary demod bytes")
    parser.add_argument("--secondary-zmq", default="tcp://127.0.0.1:55558", help="internal ZMQ address for secondary demod bytes")
    parser.add_argument("--match-window-ms", type=float, default=1000.0, help="wall-clock window for primary/secondary association")
    parser.add_argument("--expected-name", default="SDR_EXADV", help="expected Complete Local Name for event OK marker")
    parser.add_argument("--print-packets", action="store_true", help="print every decoded packet as well as matched events")
    parser.add_argument("--keep-crc-errors", action="store_true", help="keep packets with CRC mismatch")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
