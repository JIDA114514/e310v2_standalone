# python脚本说明

## BLE数据包结构

以最简单的一个广播数据包为例，BLE数据包必须具有以下结构：

前导码(Preamble) $\rightarrow$ 接入地址(Access Address) $\rightarrow$ 链路层协议数据单元(PDU) $\rightarrow$ 24位CRC校验码

![ble_header](../doc/ble_header.png)

### 前导码

固定格式：0xAA（10101010），视接入地址最低位可能会变为0x55（01010101）

> 接入地址最低位为1时，因为BLE采用小端传输和小端序，所以发送时接入地址发出的第一个比特就是这个1，为了让前导码和这个最低位之间形成跳变，就要求前导码最后发送出去的数据为0,考虑到前导码也采用了小端传输，此时应该使用0x55.

用于时钟同步和边界检测，不参与白化和CRC

### 接入地址

广播帧使用统一接入地址：0x8E89BED6

- 最低位为0,所以前导码使用0xAA
- 由于采用小端序，实际在代码中会交换字节顺序；
- 不参与白化和CRC校验

### 链路层协议数据单元

包含一个Header，MAC地址以及负载

Header为16位长，控制设备之间的通信状态。

广播数据包和连接态数据包的header含义不同，此处仅列举广播数据包的规定：

header第一个字节的bits[3:0]表示广播报文类型，bits[5:4]是两位保留位，新版本蓝牙用于Chsel，最后bit[6]是TxAdd,最高位bit[7]是RxAdd

|二进制值|类型名称|含义|用途|
|:---:|:---:|:---:|:---:|
|0000|ADV_IND|可连接的非定向广播|表示设备可以被连接，也允许被扫描|
|0001|ADV_DIRECT_IND|可连接的定向广播|针对特定设备发起的快速连接请求，通常不包含额外数据|
|0010|ADV_NONCONN_IND|不可连接的非定向广播|用于发送数据（如信标），不允许连接，也不响应扫描请求|
|0011|SCAN_REQ|扫描请求|由扫描者（如手机）发送给广播者，请求获取更多数据|
|0100|SCAN_RSP|扫描响应|由广播者回复给扫描者，包含补充的广播数据（如完整设备名）|
|0101|CONNECT_IND|连接请求|由发起者（如手机）发送，请求与广播者建立连接，包含连接参数|
|0110|ADV_SCAN_IND|可扫描的非定向广播|允许被扫描，不允许直接连接|

其他编码有拓展功能，此处不再详细描述

- TxAdd为0时表示广播包中的发送者MAC地址为厂商烧录，全球唯一，不可修改；为1时表示地址为随机地址，是软件生成的
- RxAdd为0时表示广播包中的目标设备MAC地址为公共地址，为1时表示目标是随机地址
- header的第二个字节为后面pdu的长度，广播数据包的长度不能超过32字节
- MAC地址固定6字节，在开启隐私功能后会不断切换，对于广播包则是固定的

综上，我们的报头为0x42,2对应ADV_NONCONN_IND，4表示将TxAdd置1

BLE的广播数据采用LTV结构(length-type-value)，先是一个字节表示长度（不包含自己），一个字节表示类型，一个字节表示数值。

常见的广播类型：

|十六进制值|名称|含义|
|:---:|:---|:---:|
|0x01|Flags|广播标志，定义设备的发现模式|
|0x02|Incomplete List of 16-bit Service UUIDs|不完整的16位服务UUID列表|
|0x03|Complete List of 16-bit Service UUIDs|完整的16位服务UUID列表|
|0x08|Shortened Local Name|缩略设备名称|
|0x09|Complete Local Name|完整设备名称|
|0x0A|Tx Power Level|发射功率|
|0xFF|Manufacturer Specific Data|厂商自定义数据|

广播标志主要有以下内容：

- bit0：LE有限发现模式，表示临时可见，通常用于设备刚上电配对时，不会长时间停留。
- bit1：LE通用发现模式，表示持续可见，设备始终在线，随时等待被扫描和连接
- bit2：不支持BR/EDR,置1表示仅支持BLE
- bit3：同一设备控制器能力，置1表示该设备控制器层同时支持LE和BR/EDR
- bit4：同一设备主机能力，置1表示该设备主机层同时支持LE和BR/EDR
- 其他位为保留位

一般将包含flag信息的adv内容设置为：0x020106，0x02为长度，0x01表示flags类型，0x06表示LE通用发现模式，不支持BR

### 24-CRC计算

```python
crc = 0xaaaaaa  # 广播信道初始种子，0xaaaaaa为0x555555的翻转
for b in pdu:
    for i in range(8):
        bit = (b >> i) & 1
        crc_lsb = crc & 1
        crc >>= 1
        if bit ^ crc_lsb:
            # 0xDA6000 是 BLE 多项式 0x00065B 翻转后的 LSB First 异或值
            crc ^= 0xda6000

crc_bytes = bytes([crc & 0xFF, (crc >> 8) & 0xFF, (crc >> 16) & 0xFF])
```
翻转种子的操作取决于pdu是否翻转，CRC计算是将初始值与位序翻转后的负载进行异或，最后计算得到的crc要按照小端序接到负载后面。

### 白化

白化用于将连续的0或1打断，本质上是生成一个随机序列与负载异或，只要发送端和接收端生成的序列相同，两次异或就能恢复出原始数据。

白化的实现基于一个7位的线性反馈移位寄存器（LFSR）实现，多项式选择保留第0位和第4位作为反馈多项式(0x11)。LFSR每次移位后，如果最高位为1,就将反馈多项式与原值异或得到新多项式，然后将当前负载对应的数据位翻转，再继续下一次移位。

LFSR的正序处理是将7位寄存器的最高位置1,然后取当前数据包所在的信道号的低6位拼接而成。

但此处白化处理的负载已经按照小段序重排字节序，所以是将通道号进行比特序翻转后，将第二位置一。

```python
# Swap bits of a 8-bit value
def bt_swap_bits(value):
    return (value * 0x0202020202 & 0x010884422010) % 1023

# (De)Whiten data based on BLE channel
def bt_dewhitening(data, channel):
    ret = []
    lfsr = bt_swap_bits(channel) | 2
    
    if not data:
        return ret
    first_element = data[0]
    if isinstance(first_element, str):
        # 假设所有元素都是字符串
        processed_data = [bt_swap_bits(ord(d[:1])) for d in data]
    elif isinstance(first_element, int):
        # 假设所有元素都是整数
        processed_data = [bt_swap_bits(d) for d in data]
    else:
        raise ValueError("输入列表中的元素必须是字符串或整数")

    for d in processed_data:
        for i in [128, 64, 32, 16, 8, 4, 2, 1]:
            if lfsr & 0x80:
                lfsr ^= 0x11
                d ^= i

            lfsr <<= 1
            i >>= 1
        ret.append(bt_swap_bits(d))

    return ret
```
