# RS-485 链路与协议层 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 本计划按当前会话顺序执行，不默认分派子代理。

**Goal:** 在 VibraNode 固件中实现已冻结 RS-485 协议的探头侧完整命令层：UART DMA 链路、帧编解码/CRC-32C、会话与命令状态机、FETCH 上传窗口，先 1 Mbps 后 5 Mbps 协商。

**Architecture:** 三层组件，边界对齐仓库既有分层：`probe_uart`（板级传输，持有 DMA 缓冲与方向收尾）→ `probe_frame`（无状态帧编解码 + 增量解析器）→ `probe_proto`（会话/命令/窗口状态机，消费 `probe_acquisition` 的批次队列）。全程主循环轮询驱动，不新增 ISR；USART1 RX 使用 DMA1 CH5 环回模式硬件环形缓冲，TX 使用 CH6 单次传输 + USART TDC 标志收尾（沿用 M2 已实测的 DIR 时序）。

**Tech Stack:** AT32F413CCU7、Keil ARMCC5、裸机事件循环、Python+Unicorn 宿主测试（沿用 `tests/run_probe_spi.py` 骨架）、PC 侧 pyserial 主机脚本充当 TAB5 角色。

**Spec:** `docs/superpowers/specs/2026-09-07-vibranode-rs485-protocol-design.md`（已冻结）。行为上位依据为探头设计第 8～10 节。

## 2026-09-07 执行进度

- [x] Task 1 frame：CRC-32C/构造/解析器，黄金向量与重同步测试通过（提交 `2e39331`）。
- [x] Task 2 uart：循环 RX 环、DIR 收尾、改速率，测试通过（提交 `35b6926`）。
- [x] Task 3 acquisition：最后交付样本边界字段，测试通过（提交 `576c83d`）。
- [x] Task 4 proto：会话/幂等/配置/FETCH 窗口/故障注入，测试通过（提交 `9878694`）。
- [x] Task 5 接线与构建：三模式 main、uvprojx 新组，完整 Rebuild 0 错误 0 警告（提交 `fec3d45`）。
- [x] Task 6 pc_host.py、HARDWARE_CHECK_M4、里程碑文档（软件部分）。
- [ ] 上板验证（M4 硬件项）：见 `docs/HARDWARE_CHECK_M4.md`，等待实板结果后勾选。

执行调整记录附于各任务末尾。规范勘误 3 处已回写冻结文档并同步里程碑。

## 范围与全局约束

- 本阶段只做 0x01–0x09、0x10–0x14；CALIBRATE/SAVE_CAL 应答 NOT_IMPLEMENTED，ATTITUDE 保留不发。
- 帧格式、类型数值、载荷编码、结果码、时限一律取自冻结编码表，不得自造数值；共享测试向量（编码表第 10 节）必须原样通过。
- TAB5 是唯一发起方；探头只在应答或 FETCH 窗口内发送。
- 组件状态集中到各文件 `static` 上下文；无函数内持久 `static` 堆积；芯片无关代码不得引用 AT32/RTT（probe_frame、流样本编码）。
- RAM 预算：RX 环 1024 + TX 1024 + 解析器约 1.1 KiB + 协议上下文 <0.5 KiB ≈ 3.6 KiB 增量，32 KB SRAM 内需在链接 map 记录。
- 硬件不在场时可完成代码、仿真测试与编译；上板项明确记为未执行。
- 开发分支 `codex/probe-foundation`；每任务独立提交，提交前跑适用仿真测试。

## 文件与接口

新增：

- `project/inc/probe_frame.h`、`project/src/probe_frame.c`：CRC-32C、帧构造、增量解析器。
- `project/inc/probe_uart.h`、`project/src/probe_uart.c`：USART1 DMA 链路。
- `project/inc/probe_proto.h`、`project/src/probe_proto.c`：协议状态机与流载荷编码。
- `tests/test_probe_frame.c`、`tests/test_probe_uart.c`、`tests/test_probe_proto.c`。
- `tests/pc_host.py`：PC 侧 TAB5 角色脚本（发现、协商、测量会话、故障注入开关）。
- `docs/HARDWARE_CHECK_M4.md`、`docs/milestones/2026-09-07-rs485-link.md`。

修改：

- `project/inc/probe_acquisition.h`、`project/src/probe_acquisition.c`：status 增加最后交付样本的 `last_sequence`/`last_read_us`（STOP 最终边界）。
- `project/src/main.c`、`project/inc/probe_validation.h`：模式 0=生产（采集+协议）、1=M2 诊断、2=M3 采集消费。
- `tests/run_probe_spi.py`：组件表增加 frame/uart/proto。
- `project/MDK_V5/VibraNode.uvprojx`：新增源文件分组。

### 关键接口（各任务实现必须一致）

```c
/* probe_frame.h */
#define PROBE_FRAME_MAX 1024U
uint32_t probe_frame_crc32c(const uint8_t *data, unsigned size);
unsigned probe_frame_build(uint8_t *out, uint8_t type, uint32_t session, uint32_t seq,
                           const uint8_t *payload, unsigned len); /* 返回总长，非法参数返回 0 */
typedef struct {
    uint8_t type;
    uint32_t session, seq;
    uint16_t payload_len;
    uint8_t payload[1005];
    /* 内部：解析状态、15 字节前缀缓冲、last_byte_us、计数 */
} probe_frame_parser;
void probe_frame_parser_init(probe_frame_parser *p, uint64_t now_us);
/* 返回 1 表示 parser 内有一帧完整合法帧待处理（type/payload 已就绪），0 否则 */
int probe_frame_parse(probe_frame_parser *p, const uint8_t *data, unsigned size, uint64_t now_us);

/* probe_uart.h */
void probe_uart_init(void);                    /* 1 Mbps、RX 环武装、DIR 接收 */
void probe_uart_set_rate(uint32_t bps);        /* 仅待机；重置接收环 */
unsigned probe_uart_read(uint8_t *out, unsigned capacity); /* 排空环内新字节 */
int probe_uart_send(const uint8_t *frame, unsigned size, uint32_t timeout_us); /* 忙/非法返回 0 */
void probe_uart_poll(void);                    /* TX 完成→DIR 拉低；超时/错误计数并复位接收 */
uint32_t probe_uart_error_count(void);

/* probe_proto.h */
void probe_proto_init(void);
void probe_proto_poll(void); /* 主循环唯一入口：读环→解析→分发→窗口服务→TX 收尾 */
```

模式宏：`PROBE_HARDWARE_VALIDATION` 0=生产（采集+协议）、1=M2 诊断、2=M3 采集消费（`probe_validation.h` 内 `#ifndef` 默认 0）。

## Task 1：probe_frame —— CRC-32C、帧构造、增量解析器

**Files:** 新增 `project/inc/probe_frame.h`、`project/src/probe_frame.c`、`tests/test_probe_frame.c`；修改 `tests/run_probe_spi.py`（组件 "frame"：仅 test + probe_frame.c，include 加 `project/inc`）。

**Interfaces:** 产出上方 probe_frame.h 全部接口；后续任务依赖 `probe_frame_parser` 字段名与 `probe_frame_build` 返回值语义。

- [ ] 写失败测试 `tests/test_probe_frame.c`，断言以行为号返回 0 表示通过（骨架沿用 test_probe_spi.c）：
  - CRC 自检 `"123456789"` → 0xE3069283；
  - 编码表第 10 节帧 A 十六进制串逐字节 `probe_frame_build` 重建比对，并 `probe_frame_parse` 后校验 type/session/seq/载荷；
  - 帧 B 同样构造比对 + 解析（含 0xFFF5D8 负数样本字节）；
  - 帧前注入 32 字节垃圾 + 半个帧头 `A5 5A C3`，解析器应跳过并锁定后随完整帧；
  - 坏 CRC 帧：计数 +1，继续解析下一帧不卡死；
  - 载荷长度 > 1005 或超类型上限（如 SET_CONFIG 请求 > 13）：丢弃不交付；
  - 未知类型 0x7F：丢弃不交付；
  - 帧内 51 ms 字节间隔：解析复位，残帧后随的新帧可交付；
  - `probe_frame_build(len=1006)` 返回 0。
- [ ] `python tests/run_probe_spi.py --frame` 确认编译失败/断言失败（未实现）。
- [ ] 实现 `probe_frame.c`：CRC 用无表位循环（8 步/字节；每 5 ms 窗口约 1.6 KB 数据 ≈ 0.1 ms 量级，留注释，若实测超预算再换 256 项表）；解析器状态 `SCAN_HEADER→PREFIX→PAYLOAD`；扫描窗口 1024 字节；类型上限表只放 RX 方向 0x01–0x09（8/8/13/4/5/4/冻结前 0/0/4，CALIBRATE/SAVE_CAL 载荷上限冻结前为 0 → 长度非 0 直接丢弃）。
- [ ] 测试通过后提交 `feat: add RS485 frame codec with CRC-32C and incremental parser`。

**执行调整（2026-09-07）**：类型与按类型载荷上限校验从解析器移到协议层——帧 B 黄金向量是 STREAM 类型，探头的解析器若只放行 0x01–0x09 就无法用它自测；且规范第 2.5 节的 `bad_type` 本就是接收角色的链路计数（GET_STATUS 字段）。解析器只负责结构（扫描/1005 上限/连续性/CRC），全部 CRC 合法帧都交付，角色校验在 Task 4 的 probe_proto 完成。帧定长前缀为 15 字节（帧头 4 + 类型/长度/会话/序号 11），`fixed[15]`。

## Task 2：probe_uart —— USART1 DMA 链路

**Files:** 新增 `project/inc/probe_uart.h`、`project/src/probe_uart.c`、`tests/test_probe_uart.c`；修改 `tests/run_probe_spi.py`（组件 "uart"：test + probe_uart.c）。

**Interfaces:** 产出上方 probe_uart.h 接口。测试文件按 test_probe_spi.c 模式自带伪 `usart_*`/`dma_*` 函数与伪寄存器（含可编程 `fake_fdt6/fake_tdc/fake_rx_count` 与 RX 字节注入点）。

- [ ] 写失败测试：
  - `send` 拷贝进 TX 缓冲、置 DIR 高、清 TDC、武装 CH6；忙时再次 `send` 返回 0；
  - `poll` 在 `FDT6+TDC` 后拉低 DIR、回到空闲；只有 FDT6 无 TDC（位还在移出）不收尾；
  - 超时（注入 `probe_time_us` 前跳）：关通道、复位 1 Mbps 接收、DIR 低、错误计数 +1；
  - RX：注入字节使环计数前移，`read` 按序取出且不重复；环写满未取（超 1024）：溢出计数 +1 并复位读指针；
  - `set_rate(5000000)` 后 `usart_init` 收到 5000000 且接收环重新武装；再次 `set_rate(1000000)` 恢复。
- [ ] 确认失败后实现：CH5 `loop_mode_enable=TRUE` 1024 字节环；写位置 = 1024 − `dma_data_number_get`；TX 沿用 M2 `probe_link_check` 序列（usart_enable FALSE 改速率、清 STS/DT、`usart_flag_clear(TDC)`、DIR 高、CH6、DMA TX 使能）；错误位 `PERR|FERR|NERR|ROERR` 按读 STS/DT 序列清除并计数。
- [ ] 测试通过后提交 `feat: add UART1 DMA link layer with circular RX and DIR timing`。

**执行调整（2026-09-07）**：TX 超时只中止本次传输、拉低 DIR 并计数，不回退 1 Mbps——速率是 SET_LINK 建立的会话状态，归协议层处置；链路层擅自回退会与协议层冲突。计划原文"复位 1 Mbps 接收"沿用自 M2 手动测试语义，不适用于生产链路。RX 环满通过"新增字节 + 未读字节 > 环容量"判定（DMA 计数回绕取模），整环作废并计数。

## Task 3：acquisition 边界字段 —— 最后交付样本

**Files:** 修改 `project/inc/probe_acquisition.h`、`project/src/probe_acquisition.c`、`tests/test_probe_acquisition.c`。

**Interfaces:** `probe_acquisition_status` 增加字段 `uint64_t last_sequence; uint64_t last_read_us;`，`accept_batch` 内更新；对外无新函数。

- [ ] 在 `test_probe_acquisition.c` 增加断言：接受批次后 status 反映该批 `first_sequence + samples - 1` 与 `read_started_us`；停止后保持。
- [ ] 实现字段更新；跑 `python tests/run_probe_spi.py --acquisition` 通过；提交 `feat: record last delivered sample boundary in acquisition status`。

## Task 4：probe_proto —— 会话、命令与 FETCH 窗口状态机

**Files:** 新增 `project/inc/probe_proto.h`、`project/src/probe_proto.c`、`tests/test_probe_proto.c`；修改 `tests/run_probe_spi.py`（组件 "proto"：test + probe_proto.c + probe_frame.c + probe_uart.c + probe_acquisition.c + adxl357/icm42688 组件）。

**Interfaces:** 对外仅 `probe_proto_init/poll`。上下文（文件内 `static`）包含：会话ID、TX 序号、请求幂等缓存（type+reqid+result 单条）、配置映射后的 `probe_acquisition_config` + config_id、窗口状态（reqid/预算已用/截止/轮转流/上一批段号）、TIME_INFO 与 STATUS 下次发送时刻、链路计数（crc/bad_type/bad_session/busy_rejects/last_result/速率回退计数）、测量态。

- [ ] 写失败测试（复用 test_probe_acquisition 的伪寄存器骨架 + Task 2 的伪 UART；驱动方式：向解析器注入请求帧后调 `probe_proto_poll`，从伪 UART TX 缓冲取响应帧断言）：
  - HELLO（会话 0）→ 采纳新会话、应答帧会话/类型/身份字段正确、`SET_CONFIG` 后 `GET_STATUS` 报告该 config_id；
  - 同 (type, reqid) 重发：结果一致且 `probe_acquisition_start` 只被调用一次；
  - SET_CONFIG 非法 ODR 码 → INVALID_PARAM；滤波码非 0 → UNSUPPORTED_CONFIG 附流ID；合法默认档回读码值与映射表一致（4 kSPS/±10 g/8 kSPS/±16 g/±500 dps）；
  - 未配置 START → NOT_READY；配置后 START → OK 且采集进入运行；测量中 SET_CONFIG → BUSY；
  - STOP → 采集停止、应答含两流 `last_sequence/last_read_us`、重复 STOP 幂等同边界；
  - FETCH：注入两流各一批（ADXL 原始 9B 样本直通、ICM 用 16B 大端包经 `icm42688_decode_fifo` 转 13B 小端样本——单测内直接构造 4 组已知值验证字节序），预算 1600 内按 轮转 发流帧 + 段首 TIME_INFO + 500 ms STATUS + 末帧 FETCH_DONE；额度用尽提前 DONE；无数据立结束；窗口内 SET_CONFIG → BUSY、STOP 允许；
  - 时间推进注入：`poll` 在 20 ms 截止强制收尾；500 ms 无 STATUS 不发；
  - 坏会话请求：不产生任何 TX、bad_session+1；未知类型：bad_type+1；0x07/0x08 → NOT_IMPLEMENTED；
  - SET_LINK 5000000（待机）→ 应答后注册"100 ms 静默后切速率"，时钟推进后 `usart_init` 收到 5 Mbps；测量中 → BUSY。
- [ ] 确认失败后实现 `probe_proto.c`：
  - 分发表按类型 switch；应答载荷恒为 请求ID(4)+结果码(1)+附加字段；
  - 窗口服务顺序：TIME_INFO（段首或 1 s 到期）→ STATUS（500 ms 到期）→ 流帧（ADXL/ICM 轮转，peek 到批次且整帧含 19 字节开销放入剩余额度才发，发后 release）→ 额度尽/无数据/20 ms 截止 → FETCH_DONE（预算内放不下也必发，used 记超出）；
  - 周期字段：Q16.16 = `(uint32_t)(((uint64_t)1000000 << 16) / odr_hz)`；质量位 = TIME_UNCERTAIN 恒置 | 批次 SEGMENT_START/NEAR_FULL_SCALE 映射；
  - SET_LINK 切换：记录待切速率与 `switch_at_us = TX 完成 + 100 ms`，`poll` 到点调 `probe_uart_set_rate`；切速后 3 s 无有效帧回退 1 Mbps（会话保留，计数）。
- [ ] 测试通过后提交 `feat: add RS485 protocol state machine with fetch window`。

**执行调整（2026-09-07）**：① 实现核对发现冻结规范第 4 节类型表另有 SET_CONFIG 请求上限 13→17 的算术错误（请求ID4+配置ID4+流数1+条目8），已随本任务勘误；类型/载荷校验按第 9 节归协议层。② SET_CONFIG 错误应答中仅 UNSUPPORTED_CONFIG 附流编号（5 字节），其余为裸结果（规范行为）。③ 幂等缓存改为保存完整应答载荷（≤32 字节）：规范第 9.1 节的意图是重试可恢复结果，仅存结果码会让 STOP 重试拿不回边界。④ TIME_INFO 在窗口内首个 ICM 流帧之后发送（锚点取自该批次的传感器 tick/IRQ 时间），此后每段一次、每 1 s 一次；先于流帧发送不可行，因锚点随首个 ICM 批次才产生。⑤ STATUS_NOTIFY 的 500 ms 节拍为探头级，不随窗口重置。⑥ 幂等重放对所有变更类请求生效，SET_LINK 同速率请求直接 OK 不再排程切换。

## Task 5：整机接线与 Keil 构建

**Files:** 修改 `project/src/main.c`、`project/inc/probe_validation.h`、`project/MDK_V5/VibraNode.uvprojx`。

- [ ] `probe_validation.h`：`#ifndef PROBE_HARDWARE_VALIDATION #define 0`；注释写明 0/1/2 三态。
- [ ] `main.c`：1=M2 诊断（原样）；2=`probe_acquisition_init+probe_capture_check_init`（原 M3 #else 内容）；0=生产 `probe_acquisition_init+probe_proto_init`；`while(1)` 同构。
- [ ] uvprojx 增加 probe_frame/probe_uart/probe_proto 源与（如缺）include 路径；确认 capture_check 仍编译（模式 2 可用）。
- [ ] UV4 全量 Rebuild：`C:\Keil_v5\UV4\UV4.exe -b project\MDK_V5\VibraNode.uvprojx -t VibraNode -o m4-build.log`，要求 0 错误 0 警告；从 map 记录 Code/RO/RW/ZI 与栈预留，对照 RAM 预算。
- [ ] 提交 `feat: wire RS485 protocol app as production firmware entry`。

## Task 6：PC 主机脚本与硬件验证说明

**Files:** 新增 `tests/pc_host.py`、`docs/HARDWARE_CHECK_M4.md`、`docs/milestones/2026-09-07-rs485-link.md`。

- [ ] `pc_host.py`（pyserial，仅标准库+pyserial；TAB5 角色）：`--port --baud` 参数；流程：1 Mbps 1 s 周期 HELLO（随机会话）→ SET_LINK 5000000 → 100 ms 后切速 → GET_STATUS 确认 → SET_CONFIG 默认档 → START → FETCH 5 ms/1600 B 循环排流到 `--dump` 文件并打印各流样本速率/质量位 → STOP → 排空 → 退出码报告。注入选项：`--corrupt-crc N`（每 N 帧翻转 1 字节）、`--drop N`。对 `tests/` 内无法仿真的部分不做虚假断言，真实链路数据以实测为准。
- [ ] `HARDWARE_CHECK_M4.md`：烧录生产镜像（模式 0）；无 PC 对端时 RTT 应显示链路初始化与 1 Mbps 接收武装；有 USB-RS485 对端时运行 `pc_host.py` 完整会话，记录实测吞吐与 DIR 切换；5 Mbps 协商失败路径（`--baud` 限 1 Mbps 复测回退）。
- [ ] 里程碑文档记录：测试结果、构建尺寸、未上板项；提交 `test: add PC host harness and M4 hardware check`。

## 阶段边界

本阶段完成后：探头具备待联调的完整协议固件，TAB5 侧（阶段 C）按同一编码表实现；姿态、标定、Flash 保存、灯语仍属后续里程碑。5 Mbps 持续吞吐、DIR 实际波形、长时稳定性须经上板验收，不得以仿真结论宣称。
