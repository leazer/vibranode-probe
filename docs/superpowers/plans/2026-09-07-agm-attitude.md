# M7 AGM 姿态集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 把 TDK AGM 姿态算法与 200 Hz 重采样链路集成进生产固件：ICM 原始流 → 抗混叠两级 FIR → 200 Hz → AGM 六轴四元数。输出上传（ATTITUDE 0x20 载荷）随 TAB5 侧里程碑另行冻结。

**Architecture:** 采集层为姿态增加独立有界原始批次旁路队列（不占用协议队列，满则丢弃并计数，原始流不受影响）；新组件 `probe_attitude` 持有解码、两级 FIR 与 AGM 调用状态；主循环有界消费。

**Spec:** 探头设计第 6 节"姿态分支"；LibAGM 输入为 20 位满量程编码（±FSR = 1<<19），int16 原始值统一 ×16 适配（无符号左移，用乘法）。

## 2026-09-07 执行进度

- [x] Task 1 AGM 链接（0 错 0 警；路径以正斜杠规避 UV4 转义吞字）。
- [x] Task 2 姿态旁路队列（2 槽，缩自 4 槽以留 SRAM 余量）。
- [x] Task 3 重采样与 AGM 驱动（三级滤波改版：99/169/21 tap Blackman-Harris，两级方案在 100-200 Hz 输入段无法达标 60 dB，见冻结决策 2 修订）。
- [x] Task 4 接线、构建（Code=43260/ZI=26984，SRAM 余量约 5.8 KB）与里程碑。
- [ ] 上板项：WCET、栈高水位、真实振动品质、温度补偿实测。

执行修订：滤波器最终为三级 8000→1600→400→200，一级 99 tap、二级 169 tap、三级 21 tap，Blackman-Harris 窗，总群延迟约 80 ms；两级 45/23 tap 方案在输入 100–200 Hz 段仅约 -30 dB，废弃。

用户审查要求（2026-09-07）：固件仓库是独立仓库，AGM 库原先经相对路径引用工作区 references/ 目录，换环境即断链。已将必要文件复制进仓库（`middlewares/agm/`：预编译 .a、invn_algo_agm.h、invn/common/invn_types.h、TDK 许可文本、README 出处记录；`tests/agm/out_invn_agm_1.h` 官方向量），Keil 工程与两个测试 runner 全部改为仓库内路径，构建与测试回归通过，固件产物 SHA256 不变（同一份库与同一份代码）。

## 冻结决策（v1）

1. 姿态仅在 ICM ODR = 8000 SPS（默认档）启用；其余 ODR 下姿态分支空闲（无协议字段宣告，ATTITUDE 上传随其载荷冻结里程碑补齐）。500→200 等有理数重采样按设计推迟到该里程碑。
2. 采样率转换 8000→200（÷40）：两级 FIR 级联——
   - 一级 ÷10（8k→800 Hz）：Hamming 窗线性相位低通，45 tap，通带 0–25 Hz，阻带 ≥775 Hz；
   - 二级 ÷4（800→200 Hz）：23 tap，通带 0–25 Hz，阻带 ≥175 Hz；
   - 合成满足设计目标"保留约 0–20 Hz、≥100 Hz 阻带 ≥60 dB"；系数由 Python 生成（`tests/design_attitude_filters.py`），频响验证随系数生成打印留档；运行期 C 单测用正弦扫频点验衰减。
3. AGM 配置：acc_fsr=当前配置 accel_g、gyr_fsr=当前配置 gyro_dps、acc/gyr ODR 周期 5000 µs（库固定 200 Hz 输入）、六轴 mask（无磁力计）、输出 QUAT_AG。温度补偿沿用 example 候选值并标注待实测验证。
4. 姿态时间戳使用 M5 估计的批次首样本时间推算的 200 Hz 样本时刻（64 位 µs）。
5. 姿态输出 v1 仅进组件状态（最新四元数 + 有效标志 + 计数），经 RTT 诊断可读；上线协议随 0x20 载荷冻结。
6. 资源预算：库代码 ~15.6 KB（M1 实测）、系数 ~1 KB RO、两级延迟线 <0.5 KB、旁路队列 2 KB；链接后核对 32 KB SRAM 不越界。WCET 上板测量，仿真不给结论。

## Task 1：AGM 链接入主工程

- [ ] uvprojx：链接 `prebuilt/lib/libInvnAlgoAGM.a`、include `LibAGM`、C/链接 misc 加 `--wchar32`；完整构建 0 错 0 警，记录 Code/RO/ZI 增量。
- [ ] 提交 `build: link TDK AGM library into firmware target`。

## Task 2：姿态旁路队列

- [ ] acquisition：ICM 接受批次复制进 4 槽姿态旁路队列，满则丢计数；API `probe_acquisition_attitude_peek/release`；status 增加姿态丢弃计数。
- [ ] 测试：旁路与协议队列互不干扰、满丢计数、释放推进。
- [ ] 提交 `feat: add attitude tap queue to acquisition`。

## Task 3：重采样与 AGM 驱动

- [ ] `tests/design_attitude_filters.py` 生成两级系数 + 频响报告；系数入 `project/inc/probe_attitude_filters.h`。
- [ ] `probe_attitude`：解码→FIR÷10→FIR÷4→AGM process（5000 µs 节拍，估计时间戳）；状态含最新 QUAT_AG（Q30）与计数。
- [ ] 测试：10 Hz 正弦通带幅值保持（±2 dB）；200 Hz 正弦经链路衰减 ≥60 dB；静止重力输入 1000 个 200 Hz 样本后四元数范数≈1 且输出 mask 有效。
- [ ] 提交 `feat: add attitude resampler and AGM driver`。

## Task 4：接线、构建与里程碑

- [ ] main 生产分支挂 `probe_attitude_init/poll`；RTT 诊断命令回读最新四元数。
- [ ] 完整构建 0 错 0 警、资源核对、里程碑文档（WCET 与滤波实测标为上板项）。
- [ ] 提交 `feat: integrate attitude branch into production firmware`。

---

# M8 校准与 Flash 保存（同会话追加）

**v1 范围冻结：** 校准拟合、双记录 Flash 存储、上电加载为探头内部能力（仿真可验证）；六面交互式采集流程与 CALIBRATE/SAVE_CAL 线缆载荷随 TAB5 侧里程碑冻结，符合"标定不覆盖原始流，应用在分析端"的设计。

- [x] `probe_calib`：六面代数拟合 + 质量门、陀螺静止零偏、CRC-32C 双记录存储（末两个 2 KB 扇区）、掉电安全轮换、上电加载与损坏回退。
- [x] calib 仿真测试与全量回归；构建 0 错 0 警（Code=43532/ZI=27040）。
- [ ] 上板项：Flash 擦写实测、断电时序、真实六面流程。
