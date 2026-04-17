import numpy as np
from scipy import signal
import math
from scipy.signal import resample_poly
from scipy.interpolate import interp1d

# ================= 配置参数 =================
SAMPLE_RATE = 10e6      # 基带采样率 4 MSPS (建议 AD9363 使用 4M，过低滤波器可能失真)
HARDWARE_SAMPLE_RATE = 30.72e6  # Zynq 实际跑出的采样率
SYMBOL_RATE = 1e6      # BLE 1M PHY (1 Mbps)
MOD_INDEX = 0.5        # BLE 调制指数 h = 0.5 (频偏 ±250kHz)
BT = 0.5               # 蓝牙标准高斯滤波器 BT 值
AMPLITUDE = 9830      # I/Q 信号幅度 (16-bit 满量程是 32767，留点余量)
SPS = int(HARDWARE_SAMPLE_RATE / SYMBOL_RATE) # 每个符号的采样点数 (这里是 4)

# 为了计算方便，我们先在一个高精度的虚拟采样率上生成完美的波形
# 找一个 1M 和 7.5M 的公倍数，比如 30 MHz (SPS=30)
VIRTUAL_SPS = 100
VIRTUAL_SAMPLE_RATE = SYMBOL_RATE * VIRTUAL_SPS
def bytes_to_bits(byte_array):
    """将字节数组转换为 BLE 标准的 LSB-first 比特流"""
    bits = []
    for b in byte_array:
        for i in range(8):
            bits.append((b >> i) & 1)
    return bits

def ble_whiten(data_bits, channel_index):
    """
    蓝牙标准数据白化 (Whitening)
    LFSR 多项式: x^7 + x^4 + 1
    初始化种子: [1, c5, c4, c3, c2, c1, c0] (c 为信道编号)
    """
    # 初始状态：最高位固定为1，低6位为信道号
    state = 0x40 | (channel_index & 0x3F)
    whitened = []
    for bit in data_bits:
        # 输出位是输入位异或 LFSR 的最低位 (x^0)
        out_bit = bit ^ (state & 1)
        whitened.append(out_bit)
        # 计算新位：x^4 ^ x^0
        new_bit = ((state >> 4) & 1) ^ (state & 1)
        # 移位并插入新位到最高位 (x^6)
        state = (state >> 1) | (new_bit << 6)
    return whitened

def gfsk_modulate(bits):
    """通过时间步进法将 0/1 比特流进行 GFSK 调制，输出 IQ 复数信号"""
    # 1. 符号映射: 1 -> +1, 0 -> -1
    symbols = np.array([1 if b == 1 else -1 for b in bits])
    
    # 2. 上采样 (在符号之间插零)
    up_symbols = np.zeros(int(len(symbols) * SPS))
    # up_symbols[::SPS] = symbols
    
    # 3. 设计高斯滤波器
    span = 3 # 滤波器跨度（符号数）
    t = np.arange(-span*SPS, span*SPS + 1) / SPS
    h_gauss = (math.sqrt(2*math.pi) / (math.sqrt(math.log(2)))) * BT * np.exp(-2 * (t * math.pi * BT / math.sqrt(math.log(2)))**2)

    h_gauss = (h_gauss / np.sum(h_gauss)) * SPS # 归一化
    for i,sym in enumerate(symbols):
        up_symbols[int(i * SPS)] = sym
    
    # 4. 滤波并生成频率轨迹
    freq_dev = np.convolve(up_symbols, h_gauss, mode='same')
    
    # 5. 频率积分得到相位轨迹
    # 相位累加系数 = (pi * h / SPS)
    phase_coeff = math.pi * MOD_INDEX / SPS
    phase = np.cumsum(freq_dev) * phase_coeff
    
    # 6. 生成 I/Q
    I = AMPLITUDE * np.cos(phase)
    Q = AMPLITUDE * np.sin(phase)
    
    return I, Q

def generate_ble_iq(bits):
    # 1. 映射为 NRZ 极性: 1 -> 1, 0 -> -1
    symbols = np.array(bits) * 2 - 1
    
    # 2. 插零上采样 (使用虚拟的高精度 SPS=30)
    up_symbols = np.zeros(len(symbols) * VIRTUAL_SPS)
    up_symbols[::VIRTUAL_SPS] = symbols
    
    # 3. 设计高精度的高斯滤波器
    span = 3
    t = np.arange(-span * VIRTUAL_SPS, span * VIRTUAL_SPS + 1) / VIRTUAL_SPS
    h_gauss = (math.sqrt(2*math.pi) / (math.sqrt(math.log(2)))) * BT * np.exp(-2 * (t * math.pi * BT / math.sqrt(math.log(2)))**2)
    # 补偿能量
    h_gauss = (h_gauss / np.sum(h_gauss)) * VIRTUAL_SPS 
    
    # 4. 高斯滤波 (平滑的频偏曲线)
    freq_dev = np.convolve(up_symbols, h_gauss, mode='same')
    
    # 5. 相位积分 (完美等效于 250kHz 频偏)
    phase_coeff = math.pi * MOD_INDEX / VIRTUAL_SPS
    phase = np.cumsum(freq_dev) * phase_coeff
    
    # 6. 生成高精度 I/Q
    i_virtual = np.cos(phase)
    q_virtual = np.sin(phase)
    
    num_samples_target = int(len(i_virtual) * (HARDWARE_SAMPLE_RATE / VIRTUAL_SAMPLE_RATE))
    
    i_target = resample_poly(i_virtual, up=192, down=625)
    q_target = resample_poly(q_virtual, up=192, down=625)
    
    # 8. 缩放到 16-bit 整数 (留出裕量防溢出)
    i_out = np.int16(i_target * 10000)
    q_out = np.int16(q_target * 10000)
    
    return i_out, q_out

def generate_ble_iq_inter(bits):
    """插值法生成信号"""

    Fs = 30.72e6  
    Rs = 1e6      
    SPS_HIGH = 100
    BT = 0.5
    MOD_INDEX = 0.5
    AMPLITUDE = 10000
    
    # 1. 符号映射
    symbols = np.array(bits) * 2 - 1
    
    # 【核心优化】：手动在头尾各延长 3 个符号的稳态边界！
    # 这样高斯滤波器在边界处能吃到稳定的历史数据，消除爬升畸变
    pad_len = 3
    padded_symbols = np.concatenate(([symbols[0]]*pad_len, symbols, [symbols[-1]]*pad_len))
    
    # 2. 上采样
    up_symbols = np.zeros(len(padded_symbols) * SPS_HIGH)
    up_symbols[::SPS_HIGH] = padded_symbols
    
    # 3. 构造高斯核
    span = 3
    t = np.arange(-span * SPS_HIGH, span * SPS_HIGH + 1) / SPS_HIGH
    h_gauss = (math.sqrt(2*math.pi) / math.sqrt(math.log(2))) * BT * np.exp(-2 * (t * math.pi * BT / math.sqrt(math.log(2)))**2)
    h_gauss = (h_gauss / np.sum(h_gauss)) * SPS_HIGH
    
    # 4. 卷积
    freq_dev_padded = np.convolve(up_symbols, h_gauss, mode='same')
    
    # 【核心优化】：切掉刚才补的头尾，精准取回真实数据的长度！
    freq_dev = freq_dev_padded[pad_len * SPS_HIGH : -pad_len * SPS_HIGH]

    # max_dev = np.max(np.abs(freq_dev))
    # freq_dev_calibrated = freq_dev / max_dev
    
    # 5. 积分计算展开相位 (Unwrapped Phase)
    phase_coeff = math.pi * MOD_INDEX / SPS_HIGH
    phase_high = np.cumsum(freq_dev) * phase_coeff
    
    # 6. 三次样条极高精度插值
    t_high = np.arange(len(phase_high)) / (Rs * SPS_HIGH)
    
    # 注意：为了绝对防止浮点误差导致 target 时间超出 high 时间，用 round
    num_target_samples = int(round(len(bits) * (Fs / Rs)))
    t_target = np.arange(num_target_samples) / Fs
    
    interpolator = interp1d(t_high, phase_high, kind='cubic', fill_value="extrapolate")
    phase_target = interpolator(t_target)
    
    # 7. 无情折叠回 IQ 域
    I = AMPLITUDE * np.cos(phase_target)
    Q = AMPLITUDE * np.sin(phase_target)

    return I, Q

def pack_for_dma(I, Q):
    """打包为 DMA 需要的 32-bit 十六进制数组，并重复两次适配 2T2R"""
    dma_array = []
    for i_val, q_val in zip(I, Q):
        # 限制范围
        i_val = int(max(-32768, min(32767, round(i_val))))
        q_val = int(max(-32768, min(32767, round(q_val))))
        
        # 处理负数
        i_hex = i_val & 0xFFFF
        q_hex = q_val & 0xFFFF
        
        # 拼装成 32-bit (低 16 位 I，高 16 位 Q)
        word = (q_hex << 16) | i_hex
        # word = (i_hex << 16) | q_hex
        
        # 【关键】重复写入两次，喂饱 FPGA 64-bit 宽度的 2T2R 接口！
        dma_array.append(word) # TX1
        dma_array.append(word) # TX2
        
    return dma_array

def generate_ble_packet(mac_hex, name_str):
    """
    自动生成合法的 BLE ADV_NONCONN_IND 广播包 (包含前导码、接入地址、报头、数据、CRC)
    """
    # 1. 固定的前导码和接入地址 (小端序)
    # preamble_acc = bytes.fromhex("AA D6BE898E")
    
    # 2. 构造负载数据 (AdvData)
    mac_bytes = bytes.fromhex(mac_hex)[::-1] # MAC地址反转为小端序
    
    flags = bytes.fromhex("020106") # 固定的 Flags
    
    name_bytes = name_str.encode('utf-8')
    name_len = len(name_bytes) + 1  # 长度 = 名字字节数 + 1字节类型(0x09)
    name_struct = bytes([name_len, 0x09]) + name_bytes
    
    payload = mac_bytes + flags + name_struct
    
    # 3. 构造报头 (Header)
    pdu_type = 0x42  # ADV_NONCONN_IND, TxAdd=1
    pdu_length = len(payload) # 自动计算真实长度！(比如 18),注意不能超过37
    header = bytes([pdu_type, pdu_length])
    
    # PDU = Header + Payload
    pdu = header + payload
    
    # 4. 计算 24-bit BLE CRC (极度硬核的底层算法)
    crc = 0xaaaaaa  # 广播信道初始种子，为0x555555的翻转
    for b in pdu:
        for i in range(8):
            bit = (b >> i) & 1
            crc_lsb = crc & 1
            crc >>= 1
            if bit ^ crc_lsb:
                # 0xDA6000 是 BLE 多项式 0x00065B 翻转后的 LSB First 异或值
                crc ^= 0xDA6000 
                
    # 把算出的 24位 CRC 转成 3 字节 (小端序)
    crc_bytes = bytes([crc & 0xFF, (crc >> 8) & 0xFF, (crc >> 16) & 0xFF])
    
    # 5. 拼装最终的空口 Raw 报文
    full_packet = pdu + crc_bytes
    
    return full_packet

def simulate_ble_demod(I, Q, original_bits_len):
    Fs = 30.72e6  
    Rs = 1e6      
    SPS = Fs / Rs 
    
    # 1. 还原复数与相位
    complex_sig = np.array(I) + 1j * np.array(Q)
    unwrapped_phase = np.unwrap(np.angle(complex_sig))
    
    # 2. 相位求导得频率 (Hz)
    freq_dev = np.diff(unwrapped_phase) * Fs / (2 * np.pi)
    
    # 3. 【核心修复：极高精度符号同步】
    # 我们的波形峰值严格在 0, SPS, 2*SPS...
    # 使用 round 确保小数 SPS (30.72) 也能精准对齐
    sample_indices = np.round(np.arange(original_bits_len) * SPS).astype(int)
    
    # 防止因为 diff 导致最后一个点越界
    valid_mask = sample_indices < len(freq_dev)
    sample_indices = sample_indices[valid_mask]
    
    # 4. 提取最佳眼图中心的频率
    sampled_freqs = freq_dev[sample_indices]
    
    # 5. 判决 (大于0为1，小于0为0)
    demodulated_bits = (sampled_freqs > 0).astype(int)
    
    return demodulated_bits.tolist(), freq_dev

def main():
    # --- 构造一包极简的 BLE 广播测试包 ---
    # 真实场景下，你需要把正确的 MAC 地址、Adv Data 和算好的 CRC 填进来
    # 这里提供一个抓包得到的真实 BLE ADV_NONCONN_IND 原始十六进制报文
    # 包含: 接入地址 + 报头 + MAC + 数据 + CRC
    my_mac = "FF2233445566"
    my_name = "SDR_BLE"
    raw_packet = generate_ble_packet(my_mac, my_name)
    print("自动生成的蓝牙包 (Hex):")
    print(raw_packet.hex().upper())
    acc_addr_bytes = bytes.fromhex("D6BE898E")

    pdu_hex_payload = raw_packet
    
    # 转换为比特流 (LSB First)
    payload_bits = bytes_to_bits(pdu_hex_payload)
    
    # 前导码 (Preamble) 永远不参与白化，且接入地址第一位是0，前导码为 0xAA (10101010)
    preamble_bits = [0,1,0,1, 0,1,0,1]
    
    channels = [37, 38, 39]
    output_filename = "ble_waveforms.h"
    
    with open(output_filename, "w") as f:
        f.write("// Auto-generated BLE Waveforms for DMA (2T2R interleaved)\n")
        f.write(f"// Sample Rate: {SAMPLE_RATE/1e6} MSPS\n\n")
        f.write("#include <stdint.h>\n\n")
        
        for ch in channels:
            # 1. 只有 Payload 参与白化
            whitened_payload = ble_whiten(payload_bits, ch)
            
            # 2. 拼接完整的空口 Bit 流
            full_bits = preamble_bits + bytes_to_bits(acc_addr_bytes)+ whitened_payload
            
            print(f"original:{full_bits[:20]}")

            # 3. 调制生成 IQ
            # I, Q = generate_ble_iq(full_bits)
            # I, Q = gfsk_modulate(full_bits)
            I, Q = generate_ble_iq_inter(full_bits)

            bits_out, freq_trance = simulate_ble_demod(I, Q, len(full_bits))
            print(f"demoded:{bits_out[:20]}")

            # 添加静默期并补0使得数组长度为2的幂
            TOTAL_POINTS = 16384 
            silence_len = TOTAL_POINTS - len(I)

            # 3. 构造静默期 (用纯载波，防尖刺)
            I_silence = np.zeros(silence_len)
            Q_silence = np.zeros(silence_len)

            I_final = np.concatenate((I, I_silence))
            Q_final = np.concatenate((Q, Q_silence))
            
            # (可选) 在波形末尾追加几十个 0，让功放(PA)平滑关闭，防止突变产生梳状谱，但会导致相位跳变
            # I = np.append(I, np.zeros(20))
            # Q = np.append(Q, np.zeros(20))
            
            # 4. 打包为 DMA 数组
            dma_array = pack_for_dma(I_final, Q_final)
            
            # 5. 写入 C 语言头文件
            f.write(f"const uint32_t ble_iq_ch{ch}[{len(dma_array)}] __attribute__((aligned(64))) = {{\n")
            # 每行打印 8 个 hex，方便阅读
            for i in range(0, len(dma_array), 8):
                chunk = dma_array[i:i+8]
                hex_str = ", ".join([f"0x{word:08X}" for word in chunk])
                f.write(f"    {hex_str},\n")
            f.write("};\n\n")
            
            print(f"生成信道 CH{ch} 波形成功！数组长度: {len(dma_array)} words")
            
    print(f"全部生成完毕，已保存到 {output_filename}")

if __name__ == "__main__":
    main()