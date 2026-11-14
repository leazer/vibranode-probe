# M7 AGM 姿态集成

## 2026-09-08 线上出口

冻结并实现 ATTITUDE 0x20：每个 48字节载荷含配置ID、ICM段号、姿态序号、已补偿群延迟的时间戳、200Hz周期、质量位和 AGM `grv_quat_q30[0..3]`。姿态组件持有独立8槽输出队列；队列满丢新值并继续序号，原始流不受影响。协议在 FETCH 内与 ADXL/ICM 三路轮转，STOP 后可继续排空，HELLO/下次 START 清除旧姿态。

组件测试覆盖队列容量、溢出计数、序号、时间与质量位；协议测试覆盖ATTITUDE帧编码和发送窗口。PC主机可解析、保存并报告姿态帧。

全部宿主/仿真回归通过；模式0完整Rebuild 0错0警：Code45272/RO2324/RW16/ZI29176，生产镜像SRAM余量3576字节（约3.5KiB）。J-Link烧录校验成功。COM13在4Mbps、slow配置运行8秒：ICM原始样本1556、姿态1458，差值98恰为200Hz直通FIR的启动填充；姿态序号0至1457连续，段号1，CRC错误0，末四元数Q30为`(1073740371, 774929, -1504382, -512583)`。捕获文件`tests/build/com13-attitude-slow.bin`。

日期：2026-09-07。开发分支 `codex/probe-foundation`，前一里程碑 `3d600f4`。依据：[探头设计](../../../../../docs/superpowers/specs/2026-09-06-vibranode-probe-design.md) 第 6 节与 [M7 计划](../superpowers/plans/2026-09-07-agm-attitude.md)。

## 本次实现

- `libInvnAlgoAGM.a` 正式链接进 VibraNode 固件目标（`--wchar32` 全目标统一，Include/链接路径以正斜杠写入工程，规避 UV4 对 `\r` 的转义吞字）。库代码落 Flash 约 19.9 KB（Code 23464→43260）。
- 采集层新增 ICM 姿态旁路队列（2 槽，独立于协议队列，满则丢批并计 `att_drops`，原始流不受影响）。
- `probe_attitude` 组件：ICM 200/500/1000/2000/4000/8000/16000/32000 SPS 全档支持。≥1 kSPS 经过 0～5 级 63-tap 半带 FIR 至 1 kSPS，再以 99-tap FIR 降至固定 200 Hz；500 SPS 用上采样 2、FIR、降采样 5 的真 5:2 有理重采样；200 SPS 保留 200 Hz 低通。随后进入 AGM 六轴（×16 库尺度适配、5000 µs 节拍、M5 估计时间戳）并保持 QUAT_AG 状态。200 SPS 不可能消除采样前已折叠的 >100 Hz 成分。
- 滤波器系数由 `tests/design_attitude_filters.py` 生成并自带 ≥60 dB 阻带断言（实测 A −110 dB / B −110 dB / C −57→随 21 tap 达标），总群延迟约 80 ms 已记录。
- START/STOP 经协议层联动姿态分支启停；主循环有界消费旁路队列。

## 验证

- 新增 attitude 仿真测试（ARMCC + AGM 原库 + Cortex-M 仿真）：8 kSPS 下 10 Hz 通带 ±2 dB、150 Hz 输入经链路输出 <3 counts（≥50 dB，受限于 int16 输出量化噪声底）、静态 +1 g/零速率 3 s 后 QUAT_AG 有效且四元数范数 ≈1；全 8 个支持 ODR 在 1 s 静态输入内均产生有效四元数和 200 Hz 输出。
- 全部 9 个测试组件回归通过；Keil 完整 Rebuild 0 错误 0 警告：Code=43260、RO-data=2424、RW-data=16、ZI-data=26984（SRAM 32 KB 余量约 5.8 KB）。
- HEX SHA256：`8570544B1F265752DADC37CCE11E5C26E78B8D2B13EE9EA9741C32E3DC1A974C`。

## 尚未验收的边界

AGM 与 FIR 的实际 CPU 占用（WCET）、栈高水位、真实振动下的姿态品质、温度补偿候选值均未上板验证；当前滤波实测值来自仿真注入。姿态数据尚无线上出口（ATTITUDE 0x20 载荷随 TAB5 侧里程碑冻结）。
