#!/usr/bin/env python3
# coding=utf-8

import argparse
import os
import random

import numpy as np

from bsp_algorithm import bsp_algorithm
from bsp_string import bsp_string


BLE_PREAMBLE_AND_AA = [0xAA, 0xD6, 0xBE, 0x89, 0x8E]
BLE_PDU_TYPE_ADV_EXT_IND = 0x07
BLE_EXT_HDR_FLAG_ADVA = 0x01
BLE_EXT_HDR_FLAG_CTE_INFO = 0x04
BLE_EXT_HDR_FLAG_ADI = 0x08
BLE_EXT_HDR_FLAG_AUX_PTR = 0x10
BLE_EXT_ADV_MODE_NONCONN_NONSCAN = 0x00
BLE_EXT_ADV_MODE_CONNECTABLE = 0x01
BLE_AUX_PHY_LE_1M = 0x01
BLE_EXT_ADV_MAX_PDU_PAYLOAD = 255
DEFAULT_AUX_OFFSET_US = 30000
DEFAULT_SECONDARY_PRE_PAD_US = 0.0
BLE_AUX_OFFSET_UNIT_30_US_MAX_US = 245_700
BLE_MIN_AUX_FRAME_SPACE_US = 300
BLE_PRIMARY_MAX_EVENT_SPACING_US = 10000
DEFAULT_MAC = "C1:A2:A3:A4:A5:A6"
DEFAULT_LOCAL_NAME = "SDR_EXADV"
DEFAULT_SID = 0
DEFAULT_DID = None
DEFAULT_PRIMARY_CHANNELS = "39"
DEFAULT_PRIMARY_SPACING_US = 9000
DIAGNOSTIC_PROFILES = {
    "baseline-nonconn-nonscan": {
        "adv_mode": BLE_EXT_ADV_MODE_NONCONN_NONSCAN,
        "include_flags": True,
        "description": "current non-connectable/non-scannable baseline with Flags + Complete Local Name",
    },
    "no-flags-name-only": {
        "adv_mode": BLE_EXT_ADV_MODE_NONCONN_NONSCAN,
        "include_flags": False,
        "description": "non-connectable/non-scannable, secondary AdvData contains only Complete Local Name",
    },
    "connectable-advdata": {
        "adv_mode": BLE_EXT_ADV_MODE_CONNECTABLE,
        "include_flags": True,
        "description": "connectable extended advertising display diagnostic with secondary AdvData",
    },
}
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_mac(text):
    mac = [int(part, 16) & 0xFF for part in text.split(":")]
    if len(mac) != 6:
        raise ValueError("MAC must contain 6 bytes")
    return mac


def validate_static_random_address(mac):
    if (mac[0] & 0xC0) != 0xC0:
        raise ValueError("static random address must have the two most significant bits set")
    if all(byte == 0x00 for byte in mac) or all(byte == 0xFF for byte in mac):
        raise ValueError("static random address must not be all zeros or all ones")


def parse_channel_list(text):
    channels = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        channel = int(part, 0)
        if channel not in (37, 38, 39):
            raise ValueError(f"primary channel must be one of 37, 38, 39: {channel}")
        if channel in channels:
            raise ValueError(f"duplicate BLE channel: {channel}")
        channels.append(channel)
    if not channels:
        raise ValueError("at least one primary channel is required")
    return channels


def ble_channel_freq_mhz(channel):
    if channel == 37:
        return 2402
    if channel == 38:
        return 2426
    if channel == 39:
        return 2480
    if 0 <= channel <= 10:
        return 2404 + 2 * channel
    if 11 <= channel <= 36:
        return 2406 + 2 * channel
    raise ValueError(f"unsupported BLE channel: {channel}")


def whiten_ll_pdu(pdu, channel):
    crc = bsp_algorithm.bt_crc(pdu, len(pdu))
    whitened = bsp_algorithm.bt_dewhitening(pdu + crc, channel)
    return BLE_PREAMBLE_AND_AA + whitened, crc


def build_aux_ptr(channel, offset_us, ca=0, phy=BLE_AUX_PHY_LE_1M):
    if not 0 <= channel <= 36:
        raise ValueError("secondary channel must be a BLE data channel in 0..36")
    if offset_us <= 0:
        raise ValueError("aux offset must be positive")
    if offset_us < BLE_AUX_OFFSET_UNIT_30_US_MAX_US:
        if offset_us % 30 != 0:
            raise ValueError("aux offset below 245700 us must be a multiple of 30 us")
        offset_units = 0
        aux_offset = offset_us // 30
    else:
        if offset_us % 300 != 0:
            raise ValueError("aux offset at or above 245700 us must be a multiple of 300 us")
        offset_units = 1
        aux_offset = offset_us // 300
    if not 0 < aux_offset < (1 << 13):
        raise ValueError("encoded AuxOffset must fit in 13 bits and be non-zero")

    return [
        (channel & 0x3F) | ((ca & 0x01) << 6) | ((offset_units & 0x01) << 7),
        aux_offset & 0xFF,
        ((aux_offset >> 8) & 0x1F) | ((phy & 0x07) << 5),
    ]


def decode_aux_ptr(aux_ptr):
    channel = aux_ptr[0] & 0x3F
    ca = (aux_ptr[0] >> 6) & 0x01
    offset_units = (aux_ptr[0] >> 7) & 0x01
    aux_offset = aux_ptr[1] | ((aux_ptr[2] & 0x1F) << 8)
    phy = (aux_ptr[2] >> 5) & 0x07
    offset_us = aux_offset * (300 if offset_units else 30)
    return channel, ca, offset_units, aux_offset, offset_us, phy


def build_complete_local_name_adv_data(name):
    encoded = name.encode("ascii")
    ad_len = len(encoded) + 1
    if ad_len > 0xFF:
        raise ValueError("Complete Local Name AD structure is too long")
    return [ad_len, 0x09] + list(encoded)


def build_adv_data(name, include_flags=True):
    adv_data = []
    if include_flags:
        adv_data.extend([0x02, 0x01, 0x06])
    adv_data.extend(build_complete_local_name_adv_data(name))
    return adv_data


def adv_mode_name(adv_mode):
    if adv_mode == BLE_EXT_ADV_MODE_NONCONN_NONSCAN:
        return "nonconn-nonscan"
    if adv_mode == BLE_EXT_ADV_MODE_CONNECTABLE:
        return "connectable"
    return f"reserved-{adv_mode}"


def build_adi(sid, did):
    if not 0 <= sid <= 0x0F:
        raise ValueError("SID must fit in 4 bits")
    if not 0 <= did <= 0x0FFF:
        raise ValueError("DID must fit in 12 bits")
    value = ((sid & 0x0F) << 12) | (did & 0x0FFF)
    return [value & 0xFF, (value >> 8) & 0xFF]


def decode_adi(adi):
    value = adi[0] | (adi[1] << 8)
    return (value >> 12) & 0x0F, value & 0x0FFF


def packet_air_us(ll_payload):
    return len(ll_payload) * 8


def ext_header_flags(pdu):
    length = pdu[1] & 0x3F
    if length < 2:
        return 0
    return pdu[3]


def validate_ext_adv_event(primary_infos, secondary_pdu, secondary_adi, mac, adv_mode):
    validate_static_random_address(mac)
    if len(primary_infos) > 1:
        for left, right in zip(primary_infos, primary_infos[1:]):
            spacing_us = right["event_offset_us"] - left["event_offset_us"]
            if spacing_us > BLE_PRIMARY_MAX_EVENT_SPACING_US:
                raise ValueError("primary ADV_EXT_IND start spacing must be <= 10000 us")
    aux_times = set()
    for primary in primary_infos:
        flags = ext_header_flags(primary["pdu"])
        primary_adv_mode = (primary["pdu"][2] >> 6) & 0x03
        if primary_adv_mode != adv_mode:
            raise ValueError("primary AdvMode does not match selected diagnostic profile")
        if flags & BLE_EXT_HDR_FLAG_ADVA:
            raise ValueError("primary ADV_EXT_IND must not include AdvA")
        if (flags & (BLE_EXT_HDR_FLAG_ADI | BLE_EXT_HDR_FLAG_AUX_PTR)) != (
            BLE_EXT_HDR_FLAG_ADI | BLE_EXT_HDR_FLAG_AUX_PTR
        ):
            raise ValueError("primary ADV_EXT_IND must include ADI and AuxPtr")
        if primary["adi"] != secondary_adi:
            raise ValueError("primary and secondary ADI must be identical within one generated waveform")
        if primary["aux_offset_us"] % 30 != 0:
            raise ValueError("AuxPtr offset must be encodable in 30 us units")
        min_aux_offset_us = packet_air_us(primary["ll_payload"]) + BLE_MIN_AUX_FRAME_SPACE_US
        if primary["aux_offset_us"] < min_aux_offset_us:
            raise ValueError(
                f"primary ch{primary['channel']} AuxOffset must be at least packet length + T_MAFS "
                f"({min_aux_offset_us} us)"
            )
        decoded_aux = decode_aux_ptr(primary["aux_ptr"])
        if decoded_aux[4] != primary["aux_offset_us"]:
            raise ValueError("encoded AuxPtr offset does not match requested offset")
        aux_times.add(primary["event_offset_us"] + primary["aux_offset_us"])
    if len(aux_times) != 1:
        raise ValueError("all primary AuxPtr values must point to the same absolute AUX start")

    secondary_flags = ext_header_flags(secondary_pdu)
    secondary_adv_mode = (secondary_pdu[2] >> 6) & 0x03
    if secondary_adv_mode != adv_mode:
        raise ValueError("secondary AdvMode does not match selected diagnostic profile")
    if not (secondary_flags & BLE_EXT_HDR_FLAG_ADVA):
        raise ValueError("secondary AUX_ADV_IND must include AdvA")
    if not (secondary_flags & BLE_EXT_HDR_FLAG_ADI):
        raise ValueError("secondary AUX_ADV_IND must include ADI")
    if ((secondary_pdu[0] >> 6) & 0x01) != 1:
        raise ValueError("secondary AUX_ADV_IND must set TxAdd=1 for a random AdvA")


def create_ext_primary_ll_payload(mac, primary_channel, secondary_channel, aux_offset_us, sid, did, adv_mode):
    aux_ptr = build_aux_ptr(secondary_channel, aux_offset_us)
    adi = build_adi(sid, did)
    ext_header = [BLE_EXT_HDR_FLAG_ADI | BLE_EXT_HDR_FLAG_AUX_PTR]
    ext_header.extend(adi)
    ext_header.extend(aux_ptr)
    ext_payload = [len(ext_header) | ((adv_mode & 0x03) << 6)] + ext_header
    if len(ext_payload) > BLE_EXT_ADV_MAX_PDU_PAYLOAD:
        raise ValueError("primary extended advertising PDU payload is too long")

    pdu = [BLE_PDU_TYPE_ADV_EXT_IND, len(ext_payload) & 0xFF] + ext_payload
    ll_payload, crc = whiten_ll_pdu(pdu, primary_channel)
    return ll_payload, pdu, crc, aux_ptr, adi


def create_ext_secondary_ll_payload(mac, adv_data, channel, sid, did, adv_mode):
    adi = build_adi(sid, did)
    ext_header = [BLE_EXT_HDR_FLAG_ADVA | BLE_EXT_HDR_FLAG_ADI]
    ext_header.extend(reversed(mac))
    ext_header.extend(adi)
    ext_payload = [len(ext_header) | ((adv_mode & 0x03) << 6)] + ext_header + list(adv_data)
    if len(ext_payload) > BLE_EXT_ADV_MAX_PDU_PAYLOAD:
        raise ValueError("secondary extended advertising PDU payload is too long")

    pdu = [0x40 | BLE_PDU_TYPE_ADV_EXT_IND, len(ext_payload) & 0xFF] + ext_payload
    ll_payload, crc = whiten_ll_pdu(pdu, channel)
    return ll_payload, pdu, crc, adi


def get_gaussian_filter(bt, sps, span=4):
    t = np.arange(-span * sps / 2, span * sps / 2) / sps
    alpha = np.sqrt(np.log(2) / 2) / bt
    h = (np.sqrt(np.pi) / alpha) * np.exp(-((np.pi * t / alpha) ** 2))
    return h / np.sum(h)


def ble_bits_to_iq_30_72m(bits, bt=0.5, pre_pad_us=0.0, post_pad_us=1000.0):
    symbols = np.array(list(bits), dtype=np.float32) * 2 - 1
    sps_high = 768
    nrz_high = np.repeat(symbols, sps_high)
    h = get_gaussian_filter(bt=bt, sps=sps_high, span=4)
    f_sig = np.convolve(nrz_high, h, mode="same")
    phase_step = np.pi / (2 * sps_high)
    phase = np.cumsum(f_sig * phase_step)
    i_high = np.cos(phase)
    q_high = np.sin(phase)
    i_out = i_high[::25]
    q_out = q_high[::25]
    i_int = np.round(i_out * 10000).astype(int)
    q_int = np.round(q_out * 10000).astype(int)
    iq_uint32 = ((q_int & 0xFFFF) << 16) | (i_int & 0xFFFF)
    iq_uint32 = np.repeat(iq_uint32, 2)
    pre_pad_words = int(round(pre_pad_us * 1e-6 * 30_720_000)) * 2
    post_pad_words = int(round(post_pad_us * 1e-6 * 30_720_000)) * 2
    if pre_pad_words > 0:
        iq_uint32 = np.concatenate([np.zeros(pre_pad_words, dtype=iq_uint32.dtype), iq_uint32])
    if post_pad_words > 0:
        iq_uint32 = np.concatenate([iq_uint32, np.zeros(post_pad_words, dtype=iq_uint32.dtype)])
    return iq_uint32


def write_one_c_array(f, symbol_name, iq_data):
    f.write(f"const uint32_t {symbol_name}[{len(iq_data)}] __attribute__((aligned(64))) = {{\n")
    for i in range(0, len(iq_data), 8):
        chunk = iq_data[i:i + 8]
        f.write("    " + ", ".join(f"0x{int(val):08X}" for val in chunk))
        f.write(",\n" if i + 8 < len(iq_data) else "\n")
    f.write("};\n")


def write_iq_c_arrays(path, arrays, meta_lines):
    with open(path, "w", encoding="utf-8") as f:
        f.write("// Auto-generated standard BLE extended advertising waveform\n")
        f.write("// Sample Rate: 30.72 MSPS (Dual Channel Interleaved)\n")
        for item in arrays:
            f.write(f"// Array {item['symbol']}: BLE ch{item['channel']} {item['freq_mhz']} MHz\n")
        for line in meta_lines:
            f.write(f"// {line}\n")
        f.write("\n#include <stdint.h>\n\n")
        for idx, item in enumerate(arrays):
            if idx:
                f.write("\n")
            write_one_c_array(f, item["symbol"], item["iq"])


def bytes_hex(data):
    return " ".join(f"{x:02X}" for x in data)


def main():
    parser = argparse.ArgumentParser(description="Generate standard BLE extended advertising IQ waveforms")
    parser.add_argument("--mac", default=DEFAULT_MAC, help="BLE advertiser MAC")
    parser.add_argument("--name", default=DEFAULT_LOCAL_NAME, help="Complete Local Name in AUX_ADV_IND")
    parser.add_argument("--channel", type=int, default=37, help="legacy single primary BLE advertising channel")
    parser.add_argument("--primary-channels", default=DEFAULT_PRIMARY_CHANNELS, help="comma-separated primary BLE advertising channels")
    parser.add_argument("--primary-spacing-us", type=int, default=DEFAULT_PRIMARY_SPACING_US, help="start-to-start spacing between primary PDUs in one advertising event")
    parser.add_argument("--secondary-channel", type=int, default=3, help="BLE data channel encoded in AuxPtr")
    parser.add_argument("--aux-offset-us", type=int, default=DEFAULT_AUX_OFFSET_US, help="AuxPtr offset encoded in the primary ADV_EXT_IND, in us")
    parser.add_argument(
        "--timing-debug-same-channel",
        action="store_true",
        help="diagnostic mode: transmit/whiten secondary on the primary channel while keeping AuxPtr on --secondary-channel",
    )
    parser.add_argument("--sid", type=int, default=DEFAULT_SID, help="advertising SID encoded in ADI")
    parser.add_argument("--did", type=int, default=DEFAULT_DID, help="advertising DID encoded in ADI; default is random per generated waveform")
    parser.add_argument(
        "--diagnostic-profile",
        choices=sorted(DIAGNOSTIC_PROFILES.keys()),
        default="baseline-nonconn-nonscan",
        help="extended advertising phone-display diagnostic profile",
    )
    parser.add_argument(
        "--include-flags",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="override whether to include BLE Flags AD structure before Complete Local Name",
    )
    parser.add_argument("--bt", type=float, default=0.5, help="BLE Gaussian BT")
    parser.add_argument("--post-pad-us", type=float, default=1000.0, help="zero-IQ silence appended after each packet")
    parser.add_argument("--secondary-pre-pad-us", type=float, default=DEFAULT_SECONDARY_PRE_PAD_US, help="zero-IQ silence prepended before AUX_ADV_IND for RF settle")
    parser.add_argument(
        "--output",
        default=os.path.join(SCRIPT_DIR, "ble_exadv_waveform_30_72M.h"),
        help="output C header path",
    )
    parser.add_argument("--primary-symbol-name", default=None, help="C array name for primary waveform")
    parser.add_argument("--secondary-symbol-name", default=None, help="C array name for secondary waveform")
    args = parser.parse_args()

    primary_channels = parse_channel_list(args.primary_channels) if args.primary_channels else parse_channel_list(str(args.channel))
    if args.timing_debug_same_channel and len(primary_channels) != 1:
        raise ValueError("--timing-debug-same-channel requires exactly one primary channel")
    if args.primary_spacing_us < 0:
        raise ValueError("--primary-spacing-us must be non-negative")
    if args.primary_spacing_us > BLE_PRIMARY_MAX_EVENT_SPACING_US:
        raise ValueError("--primary-spacing-us must be <= 10000 us within one advertising event")
    if args.primary_spacing_us % 30 != 0:
        raise ValueError("--primary-spacing-us must be a multiple of 30 us so all AuxPtr offsets remain encodable")
    if args.secondary_symbol_name is None:
        args.secondary_symbol_name = f"ble_exadv_secondary_iq_ch{args.secondary_channel}"

    mac = parse_mac(args.mac)
    validate_static_random_address(mac)
    did = args.did if args.did is not None else random.SystemRandom().randrange(0x1000)
    profile = DIAGNOSTIC_PROFILES[args.diagnostic_profile]
    adv_mode = profile["adv_mode"]
    include_flags = profile["include_flags"] if args.include_flags is None else args.include_flags
    secondary_wave_channel = primary_channels[0] if args.timing_debug_same_channel else args.secondary_channel
    adv_data = build_adv_data(args.name, include_flags=include_flags)
    primary_infos = []
    for primary_index, primary_channel in enumerate(primary_channels):
        primary_aux_offset_us = args.aux_offset_us - primary_index * args.primary_spacing_us
        if primary_aux_offset_us <= 0:
            raise ValueError("primary spacing pushes a later primary AuxPtr offset below zero")
        symbol_name = (
            args.primary_symbol_name
            if args.primary_symbol_name is not None and len(primary_channels) == 1
            else f"ble_exadv_primary_iq_ch{primary_channel}"
        )
        primary_ll_payload, primary_pdu, primary_crc, aux_ptr, primary_adi = create_ext_primary_ll_payload(
            mac,
            primary_channel=primary_channel,
            secondary_channel=args.secondary_channel,
            aux_offset_us=primary_aux_offset_us,
            sid=args.sid,
            did=did,
            adv_mode=adv_mode,
        )
        primary_iq = ble_bits_to_iq_30_72m(
            list(bsp_string.bytes_to_bits_lsb(primary_ll_payload)),
            bt=args.bt,
            post_pad_us=args.post_pad_us,
        )
        primary_infos.append(
            {
                "channel": primary_channel,
                "symbol": symbol_name,
                "event_offset_us": primary_index * args.primary_spacing_us,
                "aux_offset_us": primary_aux_offset_us,
                "ll_payload": primary_ll_payload,
                "pdu": primary_pdu,
                "crc": primary_crc,
                "aux_ptr": aux_ptr,
                "adi": primary_adi,
                "iq": primary_iq,
            }
        )
    secondary_ll_payload, secondary_pdu, secondary_crc, secondary_adi = create_ext_secondary_ll_payload(
        mac,
        adv_data,
        channel=secondary_wave_channel,
        sid=args.sid,
        did=did,
        adv_mode=adv_mode,
    )
    validate_ext_adv_event(primary_infos, secondary_pdu, secondary_adi, mac, adv_mode)
    secondary_iq = ble_bits_to_iq_30_72m(
        list(bsp_string.bytes_to_bits_lsb(secondary_ll_payload)),
        bt=args.bt,
        pre_pad_us=args.secondary_pre_pad_us,
        post_pad_us=args.post_pad_us,
    )
    decoded_aux = decode_aux_ptr(primary_infos[0]["aux_ptr"])
    decoded_secondary_sid, decoded_secondary_did = decode_adi(secondary_adi)

    meta = [
        f"primary_channels: {','.join(str(x) for x in primary_channels)}",
        f"primary_frequencies_mhz: {','.join(str(ble_channel_freq_mhz(x)) for x in primary_channels)}",
        f"primary_spacing_us: {args.primary_spacing_us}",
        f"secondary_aux_ptr_channel: {args.secondary_channel}",
        f"secondary_wave_channel: {secondary_wave_channel}",
        f"secondary_wave_frequency_mhz: {ble_channel_freq_mhz(secondary_wave_channel)}",
        f"timing_debug_same_channel: {int(args.timing_debug_same_channel)}",
        f"same_channel_diagnostic: {int(args.timing_debug_same_channel)}",
        f"secondary_rf_whitening_channel: {secondary_wave_channel}",
        f"aux_offset_us: {args.aux_offset_us}",
        f"aux_ptr_first_primary: {bytes_hex(primary_infos[0]['aux_ptr'])}",
        f"aux_ptr_decoded: ch={decoded_aux[0]} ca={decoded_aux[1]} units={'300us' if decoded_aux[2] else '30us'} offset={decoded_aux[4]}us phy={decoded_aux[5]}",
        f"diagnostic_profile: {args.diagnostic_profile}",
        f"diagnostic_profile_description: {profile['description']}",
        f"adv_mode: {adv_mode_name(adv_mode)}",
        f"adv_mode_value: {adv_mode}",
        f"adv_data_include_flags: {int(include_flags)}",
        "primary_ext_header_flags: ADI|AuxPtr",
        "secondary_ext_header_flags: AdvA|ADI",
        f"sid: {args.sid}",
        f"did: {did}",
        f"did_source: {'cli' if args.did is not None else 'random'}",
        f"adv_address: {args.mac.upper()}",
        "adv_address_type: static_random",
        f"secondary_adi: {bytes_hex(secondary_adi)}",
        f"secondary_adi_decoded: sid={decoded_secondary_sid} did={decoded_secondary_did}",
        f"adv_data: {bytes_hex(adv_data)}",
        f"local_name: {args.name}",
        f"secondary_pdu: {bytes_hex(secondary_pdu)}",
        f"secondary_pdu_payload_bytes: {len(secondary_pdu) - 2}",
        f"secondary_crc: {bytes_hex(secondary_crc)}",
        f"primary_air_us: {packet_air_us(primary_infos[0]['ll_payload'])}",
        f"secondary_air_us: {packet_air_us(secondary_ll_payload)}",
        f"min_aux_offset_us: {packet_air_us(primary_infos[0]['ll_payload']) + BLE_MIN_AUX_FRAME_SPACE_US}",
        f"secondary_pre_pad_us: {args.secondary_pre_pad_us:g}",
        f"primary_words: {len(primary_infos[0]['iq'])}",
        f"secondary_words: {len(secondary_iq)}",
        f"post_pad_us: {args.post_pad_us:g}",
    ]
    for primary in primary_infos:
        decoded_primary_sid, decoded_primary_did = decode_adi(primary["adi"])
        meta.extend(
            [
                f"primary_ch{primary['channel']}_adi: {bytes_hex(primary['adi'])}",
                f"primary_ch{primary['channel']}_adi_decoded: sid={decoded_primary_sid} did={decoded_primary_did}",
                f"primary_ch{primary['channel']}_event_offset_us: {primary['event_offset_us']}",
                f"primary_ch{primary['channel']}_aux_offset_us: {primary['aux_offset_us']}",
                f"primary_ch{primary['channel']}_absolute_aux_start_us: {primary['event_offset_us'] + primary['aux_offset_us']}",
                f"primary_ch{primary['channel']}_aux_ptr: {bytes_hex(primary['aux_ptr'])}",
                f"primary_ch{primary['channel']}_pdu: {bytes_hex(primary['pdu'])}",
                f"primary_ch{primary['channel']}_pdu_payload_bytes: {len(primary['pdu']) - 2}",
                f"primary_ch{primary['channel']}_crc: {bytes_hex(primary['crc'])}",
            ]
        )
    if args.timing_debug_same_channel:
        meta.extend(
            [
                "aux_ptr_channel_diagnostic_only: 1",
                "diagnostic_note: same_channel_diagnostic=1; secondary RF/whitening is ch37; AuxPtr channel is diagnostic only",
            ]
        )
    output_arrays = [
        {
            "symbol": primary["symbol"],
            "iq": primary["iq"],
            "channel": primary["channel"],
            "freq_mhz": ble_channel_freq_mhz(primary["channel"]),
        }
        for primary in primary_infos
    ]
    output_arrays.append(
        {
            "symbol": args.secondary_symbol_name,
            "iq": secondary_iq,
            "channel": secondary_wave_channel,
            "freq_mhz": ble_channel_freq_mhz(secondary_wave_channel),
        }
    )
    write_iq_c_arrays(
        args.output,
        output_arrays,
        meta,
    )

    for primary in primary_infos:
        decoded_primary_sid, decoded_primary_did = decode_adi(primary["adi"])
        print(f"Primary ADV_EXT_IND: ch{primary['channel']} {ble_channel_freq_mhz(primary['channel'])} MHz")
        print(f"Primary ADI ch{primary['channel']}: {bytes_hex(primary['adi'])} SID={decoded_primary_sid} DID={decoded_primary_did}")
        print(f"Primary timing ch{primary['channel']}: event_offset={primary['event_offset_us']} us AuxPtr offset={primary['aux_offset_us']} us")
        print(f"Primary PDU ch{primary['channel']}: {bytes_hex(primary['pdu'])}")
        print(f"Primary BLE CRC ch{primary['channel']}: {bytes_hex(primary['crc'])}")
    print(f"Diagnostic profile: {args.diagnostic_profile} ({profile['description']})")
    print(f"AdvMode: {adv_mode_name(adv_mode)} value={adv_mode}")
    print(f"AdvData include Flags: {int(include_flags)}")
    print(f"Secondary AUX_ADV_IND waveform: ch{secondary_wave_channel} {ble_channel_freq_mhz(secondary_wave_channel)} MHz")
    print(f"Secondary AuxPtr channel: ch{args.secondary_channel}")
    if args.timing_debug_same_channel:
        print("Same-channel diagnostic: same_channel_diagnostic=1")
        print(f"Same-channel diagnostic: secondary RF/whitening is ch{secondary_wave_channel}")
        print(f"Same-channel diagnostic: AuxPtr channel ch{args.secondary_channel} is diagnostic only")
    print(f"AuxPtr: {bytes_hex(primary_infos[0]['aux_ptr'])} offset={args.aux_offset_us} us")
    print(f"Secondary ADI: {bytes_hex(secondary_adi)} SID={decoded_secondary_sid} DID={decoded_secondary_did}")
    print(f"AdvA: {args.mac.upper()} static_random")
    print(f"AdvData: {bytes_hex(adv_data)}")
    print(f"Secondary PDU: {bytes_hex(secondary_pdu)}")
    print(f"Secondary BLE CRC: {bytes_hex(secondary_crc)}")
    print(f"Primary air time: {packet_air_us(primary_infos[0]['ll_payload'])} us")
    print(f"Secondary air time: {packet_air_us(secondary_ll_payload)} us")
    print(f"Secondary pre-pad: {args.secondary_pre_pad_us:g} us")
    print(f"Generated {len(primary_infos)} primary arrays x {len(primary_infos[0]['iq'])} words and {len(secondary_iq)} secondary words -> {args.output}")


if __name__ == "__main__":
    main()
