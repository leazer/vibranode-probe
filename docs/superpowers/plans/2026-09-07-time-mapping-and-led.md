# M5 统一时间映射 + M6 灯语 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 探头独立部分：把各流批次映射到 TMR11 时间轴（清除 TIME_UNCERTAIN 的判定路径），并实现规范第 9 节灯语状态机。两者均不依赖 RS-485 上板联调。

**Architecture:** 时间映射为采集层内每设备的锚点估计器（不新增文件）：批次 IRQ 锚点时间 + 段内序号 → 实际周期 EMA（Q16.16）与一致性计数，收敛后批次置有效时间并清 TIME_UNCERTAIN；ICM 内部 tick 的周期比同法估计，经 TIME_INFO 的 rate/flags 暴露。灯语为独立 `probe_led` 组件，由协议层工作状态驱动。

**Spec:** 探头设计第 3 节（时间模型）、第 9 节（灯语）；协议编码表第 5/7/8 节字段不变。

## 2026-09-07 执行进度

- [x] Task 1 采集层估计器（`4048263`，拍频修正记录于任务末尾）。
- [x] Task 2 协议层消费估计值（`de02a62`，含故障位只报活动条件的修正）。
- [x] Task 3 灯语组件与接线。
- [x] Task 4 构建 0 错 0 警（Code=23464/ZI=18512）与里程碑文档。
- [ ] 上板项：真实锚点误差、估计精度、LED 观感。

## M5 冻结决策

1. 锚点 = `batch_irq_us`（ACQ_COUNT 时刻的最近一次 EXINT 时间）：ADXL 为 DRDY（每样本输出就绪），ICM 为水位 INT1。v1 把锚点视为"最新样本的输出就绪时刻"，偏差 ≤ 主循环轮询间隔（实测 ≤0.6 ms），属记录在案的近似；滤波群延迟仍按配置相关信息在分析端处理。
2. `est_first_us = anchor − (samples−1) × period_q16`（Q16.16 截断）。
3. 周期估计：仅在锚点时间跨度 ≥10 ms 时测量 `measured = ((t−t0)<<16)/(seq−seq0)`（读取节奏与锚点节奏存在拍频，短跨度会把拍频当作周期跳变）；首测播种，其后 EMA α=1/8；|measured−period|×16 ≤ period 视为一致（aligned++），否则 aligned=0 并重新播种。aligned ≥ 8（约 80 ms）→ 该设备 `time_valid=1`。同锚点瞬间的批次并入下一区间。
4. 复位：start、end_segment（段切换/缺口/错误重建）、配置变更。
5. ICM tick 映射：同法估计"每 tick 的 MCU µs"（Q16.16），`tick_valid` 独立收敛；TIME_INFO 的 rate 字段填实测值（未收敛填名义 65536），flags bit0 = tick_valid。
6. `TIME_UNCERTAIN` 清除条件 = 该设备 time_valid；未收敛批次保持置位（保守）。
7. GET_STATUS 故障位 bit7（时间不确定生效中）：测量中任一流 time_valid=0 时置位。

## M6 冻结决策

1. 颜色（低电平点亮）：待测=蓝、采集=绿、标定=紫、警告=黄、故障=红、未握手/熄灭。
2. 闪烁参数冻结：紫慢闪 1 Hz（500/500 ms）、黄闪 2 Hz（250/250 ms）、红快闪 5 Hz（100/100 ms）；常亮态无闪烁。回写探头设计第 9 节。
3. 优先级由协议层 work_state 单值决定（5>4>3>常规），LED 组件只做映射与相位。
4. `probe_led_update(state)` 主循环调用，基于 probe_time 非阻塞翻转；复位/重启后从熄灭进入当前状态。

## Task 1：采集层时间估计器

**Files:** 修改 `project/inc/probe_acquisition.h`、`project/src/probe_acquisition.c`、`tests/test_probe_acquisition.c`（tick 注入 DRDY/INT1 时间）。

- [ ] batch 增加 `first_sample_us/period_q16`，status 增加 `time_valid/tick_valid/tick_period_q16`；accept_batch 内按冻结决策计算与清位；end_segment/start 复位。
- [ ] 测试：假时钟下按固定速率注入 IRQ 时间，断言 aligned 收敛后 TIME_UNCERTAIN 清除、first_sample 随批次单调、period 与注入速率自洽；制造段切换后 aligned/valid 复位。
- [x] 提交 `feat: estimate sample timeline anchors in acquisition`（`4048263`）。

**执行调整（2026-09-07）**：① 逐对锚点测量存在读取节奏×锚点节奏的拍频，短跨度会把拍频当周期跳变——测量改为仅在锚点跨度 ≥10 ms 时进行（决策 3 已更新）。② 同锚点瞬间的批次并入下一区间。③ 修复三处"估计器在 batch 指针赋值前读写该指针"的次序缺陷，输出字段改在入队赋值时从设备状态取值；ICM tick 估计改读 device->sensor_tick。④ 仿真注入的 EXINT 间隔取 100 µs 步长的整数倍，保持锚点一致性检验有意义（真实 DRDY 间隔本就稳定）。

## Task 2：协议层消费估计值

**Files:** 修改 `project/src/probe_proto.c`、`tests/test_probe_proto.c`。

- [ ] STREAM 元信息首样本时间/周期改用批次估计值；TIME_INFO rate/flags 按收敛状态填实测值与有效位；fault_bits 增加 bit7。
- [ ] 测试改为容差断言（周期与名义差 ≤2% 或与批次一致；TIME_INFO flags bit0==1）；收敛前批次仍带 TIME_UNCERTAIN。
- [x] 提交 `feat: publish estimated timeline in stream and time info`。

**执行调整（2026-09-07）**：协议层原在 STREAM meta 恒置 TIME_UNCERTAIN 位（M4 保守行为），M5 起改为映射批次标志；未收敛批次回退 delivery 锚点并保持置位，收敛后发布 first_sample_us/period_q16。测试覆盖未收敛（bit0 置位+回退锚点）与收敛（bit0 清零+实测周期+TIME_INFO 有效位+故障位 bit7 清除）两条路径。

## Task 3：灯语组件

**Files:** 新增 `project/inc/probe_led.h`、`project/src/probe_led.c`、`tests/test_probe_led.c`；修改 `tests/run_probe_spi.py`（led 组件）、`project/src/probe_proto.c`（poll 末尾驱动 LED）、`project/MDK_V5/VibraNode.uvprojx`。

- [ ] 测试：各状态颜色组合、三种闪烁的相位翻转时刻、熄灭态、低电平点亮极性。
- [ ] 实现；proto poll 以 work_state 驱动；uvprojx 加入组。
- [ ] 提交 `feat: add status LED state machine`。

## Task 4：构建与文档

- [ ] UV4 完整 Rebuild 0 错 0 警，记录尺寸与 HEX SHA256。
- [ ] 里程碑 `docs/milestones/2026-09-07-time-mapping-led.md`；上板项（真实 DRDY 锚点误差、LED 观感）明确标注未执行。
- [ ] 提交 `docs: record M5/M6 milestone`。
