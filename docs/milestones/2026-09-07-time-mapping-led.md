# M5 统一时间映射 + M6 灯语

日期：2026-09-07。开发分支 `codex/probe-foundation`，前一里程碑 `173d550`。依据：[探头设计](../../../../../docs/superpowers/specs/2026-09-06-vibranode-probe-design.md) 第 3/9 节与 [M5/M6 计划](../superpowers/plans/2026-09-07-time-mapping-and-led.md)（含冻结决策）。

## M5 时间映射

- 采集层每设备锚点估计器：批次 EXINT 锚点（ADXL=DRDY，ICM=水位 INT1）+ 段内序号 → 实际周期 EMA（Q16.16）。测量仅在锚点跨度 ≥10 ms 时进行（读取节奏与锚点节奏存在拍频，短跨度把拍频当周期跳变——实现期发现并修正设计）；|measured−period|≤1/16 连续 8 次 → `time_valid`。
- 段切换/缺口/错误重建复位锚点与一致计数（保留已学周期）；START 按新 ODR 从名义周期重新播种。
- 批次携带 `first_sample_us/period_q16`；`time_valid` 后清除 TIME_UNCERTAIN，未收敛保持置位（保守）。
- ICM 内部 tick 周期比同法独立估计（MCU µs per tick，Q16.16）；TIME_INFO 的 rate 字段填实测值、flags bit0=tick_valid，未收敛回退名义 65536/无有效位。
- 协议层：STREAM meta 首样本时间在未收敛时回退 delivery 锚点（read_started_us），收敛后发布估计值；GET_STATUS/STATUS 的故障位 bit7（时间不确定生效中）在测量中任一流未收敛时置位。故障位同时改为只报告**活动**条件（设备停止且带故障码）——锁存的 last_fault 仅供诊断，恢复后状态回绿，符合规范"瞬时警告可恢复"。

## M6 灯语

- `probe_led` 组件：状态→颜色映射（待测蓝/采集绿/标定紫/警告黄/故障红/未握手灭，低电平点亮），闪烁参数冻结为紫 1 Hz、黄 2 Hz、红 5 Hz，状态切换后从亮相开始。
- 由 `probe_proto_poll` 以与 STATUS/GET_STATUS 相同的 work_state 驱动（单一事实来源）；init 熄灭。

## 验证

- 新增 led 测试组件（颜色/极性/闪烁相位/暗态）；acquisition 测试注入 EXINT 锚点时间断言收敛与复位；proto 测试新增收敛相位（流帧清除 TIME_UNCERTAIN、实测周期非名义、TIME_INFO 有效位与实测速率、故障 bit7 清除、LED 得到测量态驱动）。
- 全部 8 个测试组件回归通过；Keil 完整 Rebuild 0 错误 0 警告：Code=23464、RO-data=1268、RW-data=16、ZI-data=18512（SRAM 余量约 13.5 KB）。
- HEX SHA256：`ED64A28A275D6D4B7E0335E0C824E65F916C65D0ED1D1C81AB9C638DC4CCFFF4`。

## 尚未验收的硬件边界

真实 DRDY/INT1 锚点误差、实际样本周期的估计精度、LED 实际观感与闪烁频率均未上板确认；时间映射的绝对精度仍属"输出就绪时刻"模型，未宣称传感器间同步精度。当前仿真收敛值反映的是仿真节拍，不代表实测周期。
