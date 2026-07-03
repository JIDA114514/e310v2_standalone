# antsdr-no-OS
Standalone application based on ADI hdl and no_OS for ANTSDR.

主要开发基于BLE的跨技术通信（cross technology communication,CTC）

代码来自于[microphase官方](https://github.com/MicroPhase/antsdr_standalone)，需要注意hdl中用于e310开发板的vivado项目为“antsdre310"而非"e310v2"。"e310v2"的代码生成后在下板执行时可能会报错：

```
cf-ad9361-lpc: Status errors
SAMPL CLK: 61440000 tuning: RX
  0:1:2:3:4:5:6:7:8:9:a:b:c:d:e:f:
0:# # # # # # # # # # # # # # # # 
1:# # # # # # # # # # # # # # # # 
ad9361_dig_tune_delay: Tuning RX FAILED!
ad9361_init : AD936x initialization error
```

## 主要功能

- 支持BLE波形的发送，可以在37,38,39信道上发送广播包，可被商用器件或智能手机检测到。波形由python脚本生成，通过dma传输给ad9363发射。
- 支持裸机程序自动生成BLE广播包并发射，可被商用器件检测
- 支持裸机程序检测BLE广播数据包
- 通过python脚本实现BlueBee和PatternBee的软件模拟
- 可通过python脚本控制hackrf和e310实现BLE上下行流程
- 可通过python脚本实现hackrf和e310的zigbee上下行流程
