# ADXL357 低频温度遥测

用户要求 TAB5 温度从 ICM FIFO 温度切至 ADXL357，并避免挤占振动数据流。

## 合同

- 既有冻结消息及保留字节完全不动，新增可忽略的 `TEMPERATURE=0x21`，仅在已绑定会话的 FETCH 内发送。
- 16 字节 payload，小端：`config_id:u32`、`sample_uptime_us:u64`、`adxl_raw:u16`、`flags:u8`（bit0 valid、bit1 live）、`reserved:u8=0`。帧总长 35 字节，纳入 FETCH bytes_sent/frames_sent/预算。
- 上限一秒一帧；采样缓存不排队，不追补旧温度。无效读数不发送，主机采集中 3.5 秒收不到有效值则失效。新旧主机与探头互配：旧主机忽略新类型，新主机面对旧固件显示无温度，不新增轮询命令或超时。
- ADXL 采集状态机只在 FIFO 定时空隙发起温度 SPI burst，读取 0x06/0x07 后再次读取高位，丢弃不一致值。失败只影响温度有效性，不重置波形段、不改计数、配置、时间估计器或标定记录。
- STOP 后 ADXL 温度通道停机，不额外唤醒整颗芯片；保存末次值、live=0。ICM 待机姿态机制保持。
- ADI Rev.A：`T=25+(1885-raw)/9.05`；约 0.11°C/code 为标称分辨率，并非已标定绝对精度。温度是芯片温度，不等同环境温度。

## 预算

最多 35B/s、1 frame/s，相比原先约 170KB/s / 695fps，增加约 0.021% 字节、0.14% 帧。温度 SPI 每秒 5 字节（两次事务）；排队和主循环延迟仍须实机回归核对，不以数学预算代替验收。

## 验证

- 采集、协议、SPI、标定、姿态、帧封装回归通过；包括温度高位改变的撕裂读、提交失败不重置 raw segment、STOP 保留，FETCH 34/35B 边界、bytes_sent/frames_sent 和 1Hz 上限。
- 四模式 Keil 均零错误零警告；生产 Code=63364、RO=2636、RW=288、ZI=53200。镜像边界/标定区排除检查通过。
- J-Link 20721552 下载已校验；4KB 标定区前后 SHA256 一致：`ffca1184fc52e41178c8b343b581eb58a6cd1dc5a7c6e0e7404a0ee3218c1751`。
- TAB5 实机已见有效 ADXL 温度（约 33–36°C），RUN live=1、STOP live=0；用户回复“已测试”。连续 RUN 多个 5s 窗口温度更新时 CRC/timeouts/retries/overruns/queue_drops 均零，gap/rejected 累计值稳定。反复启停后 gap 累计曾增加，不能据此声称全程零间断或精确 100% 交付；启动时间线收敛仍会拒绝部分 TIME_UNCERTAIN 帧。未做长期或绝对温度标定验收。
- 设备证据在 TAB5 仓 `build/adxl-temperature-device.log`；生产 HEX SHA256：`a8f5922e39ed4516f0969f594fb39dc36021af38737726caa0fc1f2fc0139064`。

资料：https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL356-357.pdf 第 27、34、39 页。
