# M2 硬件验证说明

这是旧 M2 板级诊断流程，需显式选择 `PROBE_HARDWARE_VALIDATION=1`；当前默认模式 3，入口见 [单板 RTT 验证](HARDWARE_CHECK_RTT.md)。它不运行正式 RS-485 协议、FIFO 连续上传、姿态或校准，也不会自动写 Flash 标定区。数据寄存器快照仅用于判断芯片响应、轴向与量级，不能用于 FFT 或无丢样验收。

## 1. 烧录与 RTT

1. 用 Keil 打开 `project/MDK_V5/VibraNode.uvprojx`，选择 `VibraNode`，执行 Rebuild；使用已有 SWD 设置下载。
2. 当前生成文件为 `project/MDK_V5/objects/VibraNode.hex`。不要使用测试目录中的 AXF；AGM 全向量测试镜像超过本芯片 Flash，仅用于模拟器。
3. 给探头正常供电，连接 RTT channel 0。若错过启动输出，在 RTT 输入小写 `r` 重试传感器诊断（不重启 MCU/TMR11）。不要在 Keil 暂停后继续运行来判断 IRQ 频率；需要时复位重新测量。

预期启动日志包含：

```text
VibraNode started
HW CHECK M2: SPI1=10MHz SPI2=20MHz; r=retry l=LED test
ADXL ID=ad 1d ed rev=...
ICM WHO_AM_I=47
ADXL CONFIG PASS; diagnostic snapshots only
ICM CONFIG PASS; diagnostic snapshots only
```

大小写以 RTT 实际格式为准。ADXL 修订号不固定要求 01；前三个 ID 必须符合。ID 错误时不会继续配置该传感器。错误记录会锁存为 FAIL，另一路仍继续；排查后按 `r` 重试。

## 2. 静止与转动

每秒输出两路 `samples/errors/a_raw/g_raw` 以及中断统计。

- 快照约每 10 ms 一次，因此 `samples` 每秒大致增加 100，主循环调度会略少。它不是硬件 4/8 kSPS 的计数。
- ADXL `a_raw` 为带符号 20 bit 计数；ICM `a_raw/g_raw` 为带符号 16 bit 计数；单位尚未换算或标定。
- 静止放置：朝向重力的轴明显非零，其余轴较小；转到反面，主轴符号应反转。ICM 在 ±16 g 下约 2048 count/g，可作为粗略检查。
- 轻缓转动探头时 ICM `g_raw` 应响应，停下后回到零附近。不要用大力敲击作为首次连通检查。
- 两颗芯片的安装方向可能不同，当前日志是各自芯片坐标，不能要求对应下标必然同符号；请记录外壳方向与两路符号关系。
- 运行 1～2 分钟，记录 `errors` 是否为 0、两个计数是否持续增加，以及 RTT 是否出现 FAIL。

## 3. 中断和 TMR11

`irq=A/B/C` 依次为 PB0 ADXL DRDY、PB1 ADXL INT1、PB10 ICM INT1 的累计次数，`dt_us` 为最近两次对应中断的间隔。

- PB0：目标接近 4 kHz、间隔约 250 μs；读取寄存器会影响 DRDY 清除，统计用于初步检查，不是无丢样证明。
- PB10：目标接近 8 kHz、间隔约 125 μs。
- PB1：本诊断版本映射的是 DATA_RDY，不是正式 FIFO 水位。状态寄存器只在快照时读取清除，因此不能要求这里达到 4 kHz；有响应且持续变化即可先确认引脚路径。
- 本版没有消费 ADXL FIFO，FIFO 可能自然溢出；这是快照诊断模式的已知行为，不能将它用于生产 FIFO 验收。
- `time=高32位:低32位` 是 64 位微秒时间。正常每秒报告应约增加 1000000；运行超过数分钟时不应每约 65.536 ms 回退一次。最终时钟准确度需独立时基/仪器测量。

## 4. RGB

在 RTT 输入 `l`：依次红、绿、蓝各约 1 秒，再全部熄灭。它是人工引脚测试，不代表正式采集/故障状态机。正常诊断期间未建立 TAB5 握手，灯保持熄灭。

确认颜色与日志一致，PB3/4/5 低电平时对应灯亮；SWD 调试在灯测试时仍可使用。

## 5. 可选 RS-485 5 Mbps 测试

先确保对端不会同时主动发送。可以连接支持 5 Mbps 的收发测试端或逻辑分析仪；普通 USB 串口适配器不一定支持该速率。没有这些设备时先跳过，不影响板内 SPI 验证。

### 发送

- RTT 输入 `u`，探头临时切到 5 Mbps、8N1，发送一次 **64 个 0x55 原始字节**。
- 这不是 ASCII 文本“55”，也不是正式协议帧。
- 日志 `TX complete` 只说明 MCU 端完成；需确认对端确实收到 64 个正确字节。
- UART 波形每位约 200 ns，64 字节理想时长约 128 μs。PA8 高电平覆盖发送过程，最后停止位发出后才回低。
- 完成或失败均恢复 1 Mbps 接收。用逻辑分析仪比较 DMA 完成与 DIR 边界时，以实际 UART 最后停止位为准。

### 接收

- RTT 输入 `v`，日志出现 `RX armed` 后，测试端在 **3 秒内**以 5 Mbps、8N1 发送恰好 64 个 0x55。
- 预期 `RX complete: mismatches=0/64`，然后恢复 1 Mbps 接收。
- 未发数据时应在 3 秒后输出 timeout，不能挂住传感器日志。接收出现 UART framing/noise/overrun 或 DMA 错误时报告失败。
- `u/v` 不能同时运行，重复请求提示 busy。不要在一批测试数据之后持续发送；此功能不是连续接收器。

## 6. 请反馈的最小信息

1. `r` 后两颗身份与 CONFIG PASS/FAIL 日志。
2. 静止时连续 3～5 秒日志，以及翻面/转动后的日志。
3. `l` 三种颜色是否正确。
4. 如能测 RS-485，附 `u/v` 结果及对端接收情况。

若失败，优先附失败原文，不必先自行修改频率或寄存器；这样可以区分身份错误、回读不匹配、DMA 超时和接线/时序问题。
