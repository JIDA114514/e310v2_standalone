# 项目概述

## 总目标

本项目以论文BlueBee为基础，论文原文在/python/ctc_sim/bluebee/文件夹下。最终目标是利用BLE的拓展广播特性，将bluebee负载放在辅助通道的广播包上，以尽可能小的改动实现BLE向zigbee的跨协议通信，并实现性能测量。

## 当前目标

实现BLE拓展广播的基础功能，要求在手机上的nrf connect软件中检测到广播包内容，将设备名放在辅助信道上，需要能看到该内容。

## 阶段性实验结论

- 2026-06-23：使用`generate_ble_exadv_iq_30_72M.py --timing-debug-same-channel`生成的同信道拓展广播波形，并将裸机发送路径配置为primary和secondary都在ch37发射后，手机nRF Connect已经能看到primary包。
- 该结果说明当前ADV_EXT_IND主信道PDU、基本BLE调制链路、发射功率/频点配置至少已达到手机可检测的程度。
- 该结果不能直接证明手机已经跟随AuxPtr到真实辅助信道；`--timing-debug-same-channel`模式下secondary波形实际按ch37生成和发射，AuxPtr仍可编码ch3但只作为诊断字段。
- 同信道诊断的下一步成功判据是手机nRF Connect显示`SDR_EXADV`。若只显示`C1:A2:A3:A4:A5:A6`，只能说明手机已解析/展示secondary AdvA，不能说明secondary AdvData里的Complete Local Name已被合并到extended advertising report。
- 同信道实验记录应区分：手机是否显示地址、是否显示`SDR_EXADV`、显示延迟、是否关闭duplicate filtering或刷新扫描缓存；HackRF fixed ch37抓包应同时确认primary和secondary，且secondary包含`AdvA=C1:A2:A3:A4:A5:A6`和`Name=SDR_EXADV`。
- 2026-06-23：手机nRF Connect在同信道模式下可显示`AdvA=C1:A2:A3:A4:A5:A6`、`Advertising type=Bluetooth 5 Advertising Extension`、`Primary PHY=LE 1M`、`Data status=Complete`，但仍不显示`SDR_EXADV`。由于AdvA只在secondary AUX_ADV_IND中，说明手机已经接收到同信道secondary；HackRF fixed ch37已确认secondary AdvData包含`Name=SDR_EXADV`，因此下一步优先排查手机/nRF Connect对当前扩展广播类型或AD组合的展示策略。
- `generate_ble_exadv_iq_30_72M.py`提供`--diagnostic-profile`：默认`baseline-nonconn-nonscan`保持当前基线；`no-flags-name-only`只保留Complete Local Name；`connectable-advdata`将AdvMode改为connectable用于手机展示诊断。三者仍应先在同信道ch37模式下比较，不要同时切换到真实ch3 AUX。
- 后续排查真实拓展广播时，应优先以“单primary ch37 -> ch3 AUX”为最小变量实验，不要过早恢复ch37/ch38/ch39多主信道发送，以免同时引入多次LO切换、扫描窗口命中概率和不同AuxPtr offset等额外因素。

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

1. 工作区裸机程序代码未被git追踪
2. 裸机程序代码发生修改后，仅检查代码逻辑和语法，由用户来编译操作
3. 以实际规范为准，代码注释可能有错
4. doc/文件夹下有BLE5.1规范BLE_Core_v5.1.pdf,以此作为参考
