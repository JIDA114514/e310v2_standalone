import numpy as np
from gnuradio import gr
import pmt

class blk(gr.sync_block):
    """
    输入: uint8 比特流(每样本0/1), 来自 Binary Slicer
    输出: 透传同样比特流
    功能: 检测 BLE 广播包 (AA=0x8E89BED6), 去白化, CRC校验
    """
    def __init__(self, channel_index=37, verbose=True):
        gr.sync_block.__init__(
            self,
            name='BLE Packet Checker',
            in_sig=[np.uint8],
            out_sig=[np.uint8]
        )
        self.ch = int(channel_index)
        self.verbose = bool(verbose)

        # 缓冲区（bit）
        self.buf = []

        # BLE 广播 AA
        self.aa = 0x8E89BED6
        self.aa_bits_lsb = self._bytes_to_bits_lsb(bytes.fromhex("D6BE898E"))  # on-air LSB-first

        # 统计
        self.total_hits = 0
        self.total_crc_ok = 0

    # ---------- 基础工具 ----------
    def _bytes_to_bits_lsb(self, b):
        bits = []
        for x in b:
            for i in range(8):
                bits.append((x >> i) & 1)
        return bits

    def _bits_to_bytes_lsb(self, bits):
        n = len(bits) // 8
        out = bytearray()
        for i in range(n):
            v = 0
            for k in range(8):
                v |= (bits[i*8 + k] & 1) << k
            out.append(v)
        return bytes(out)

    def _ble_dewhiten(self, bits, channel):
        # whitening LFSR: x^7 + x^4 + 1, init = 1 | channel(6bit) at bits[6:1]? 常见实现如下：
        state = 0x40 | (channel & 0x3F)
        out = []
        for b in bits:
            wb = state & 1
            outb = b ^ wb
            out.append(outb)
            new_bit = ((state >> 4) & 1) ^ (state & 1)
            state = (state >> 1) | (new_bit << 6)
        return out

    def _ble_crc24(self, pdu_bytes):
        """
        BLE CRC24 (LSB-first), poly=0x00065B
        广播CRCInit=0x555555
        """
        crc = 0x555555
        for byte in pdu_bytes:
            for i in range(8):
                bit = (byte >> i) & 1
                c0 = crc & 1
                crc >>= 1
                if c0 ^ bit:
                    crc ^= 0x00065B
        return crc & 0xFFFFFF

    def _find_all_aa_positions(self, bits, aa_bits):
        pos = []
        m = len(aa_bits)
        # 朴素匹配，够用；后续可加汉明距离容错
        for i in range(0, len(bits) - m + 1):
            if bits[i:i+m] == aa_bits:
                pos.append(i)
        return pos

    # ---------- 主处理 ----------
    def work(self, input_items, output_items):
        inp = input_items[0].astype(np.uint8)
        out = output_items[0]
        out[:] = inp  # 透传

        # 只保留0/1
        new_bits = (inp & 1).tolist()
        self.buf.extend(new_bits)

        # 防止无限增长
        MAX_BUF = 200000
        if len(self.buf) > MAX_BUF:
            self.buf = self.buf[-MAX_BUF:]

        # 在缓冲中找AA
        aa_pos = self._find_all_aa_positions(self.buf, self.aa_bits_lsb)

        # 对每个命中点尝试解包
        # 包结构(不含preamble): AA(32) + PDU(16+payload*8) + CRC(24)
        # 先至少拿到 header(16)+crc(24)
        for p in aa_pos:
            start_payload = p + 32
            if len(self.buf) < start_payload + 16:
                continue

            # header 在白化域内，需要先取出并去白化
            hdr_whitened = self.buf[start_payload:start_payload+16]
            hdr_bits = self._ble_dewhiten(hdr_whitened, self.ch)
            hdr = self._bits_to_bytes_lsb(hdr_bits[:16])
            if len(hdr) < 2:
                continue

            pdu_type = hdr[0] & 0x0F
            length = hdr[1] & 0x3F  # BLE adv length 6bit

            # 合法长度粗检（广播PDU最大37）
            if length > 37:
                continue

            total_payload_bits = (2 + length + 3) * 8  # header + payload + crc
            end_all = start_payload + total_payload_bits
            if len(self.buf) < end_all:
                continue  # 数据还不够

            whitened_block = self.buf[start_payload:end_all]
            dewhitened_block = self._ble_dewhiten(whitened_block, self.ch)
            raw_bytes = self._bits_to_bytes_lsb(dewhitened_block)

            if len(raw_bytes) != (2 + length + 3):
                continue

            pdu = raw_bytes[:2+length]
            rx_crc_bytes = raw_bytes[2+length:2+length+3]
            rx_crc = rx_crc_bytes[0] | (rx_crc_bytes[1] << 8) | (rx_crc_bytes[2] << 16)
            calc_crc = self._ble_crc24(pdu)
            crc_ok = (rx_crc == calc_crc)

            self.total_hits += 1
            if crc_ok:
                self.total_crc_ok += 1

            if self.verbose:
                print(
                    "[BLE] hit=#{}, ch={}, pdu_type=0x{:X}, len={}, crc_ok={}, "
                    "rx_crc={:06X}, calc_crc={:06X}, pdu={}".format(
                        self.total_hits, self.ch, pdu_type, length, crc_ok,
                        rx_crc, calc_crc, pdu.hex().upper()
                    )
                )

            # 发消息端口（可选）
            # self.message_port_pub(pmt.intern("out"), pmt.to_pmt({"crc_ok": crc_ok}))

        # 为避免同一命中反复解析，裁剪到最后一段
        # 保留足够尾巴，防止跨work边界丢包
        TAIL = 4096
        if len(self.buf) > TAIL:
            self.buf = self.buf[-TAIL:]

        return len(out)