# 项目概述

## 总目标

本项目以论文BlueBee为基础，论文原文在/python/ctc_sim/bluebee/文件夹下。最终目标是利用BLE的拓展广播特性，将bluebee负载放在辅助通道的广播包上，以尽可能小的改动实现BLE向zigbee的跨协议通信，并实现性能测量。

## 当前目标

实现BLE拓展广播的基础功能，要求在手机上的nrf connect软件中检测到广播包内容，将设备名放在辅助信道上，需要能看到该内容。

# 设备

- 一台搭载zynq7020和ad9363的开发板，在主机电脑上生成好要发送的波形数据后，通过该开发板发射。
- 一台HackRF One用于辅助开发

# 项目结构

- python/ctc_sim/bluebee文件夹下的内容用于生成和分析bluebee信号，其中generate_bluebee_iq_30_72M.py将生成的zigbee前导码放在BLE的周期广播包中然后生成IQ波形数组
- python/std_ble文件夹下主要有以下几个功能：
  - ble_packet_detector.py用于控制HackRF检测生成的BLE数据包
  - ble_rx.py用于控制HackRF检测BLE广播信号
  - generate_ble_exadv_iq_30_72M.py用于生成BLE拓展广播包信号
  - generate_ble_iq_30_72M.py用于生成BLE周期广播信号
- python/ctc_sim/std_zigbee文件夹用于操纵HackRF模拟zigbee设备和生成可发送的zigbee波形数据
- hdl/projects/antsdre310/antsdre310.sdk/app/src文件夹下为开发板裸机程序代码

# 注意事项

1. 裸机程序代码未被git追踪
2. 裸机程序代码发生修改后，仅检查代码逻辑和语法，由用户来编译操作
