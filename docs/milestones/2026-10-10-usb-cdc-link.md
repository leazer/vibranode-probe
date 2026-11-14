# 2026-10-10 RS485 → USB CDC 链路替换（round-17）

## 动机

RS485 5 Mbps 已到线速天花板：round-16 窗口连发把发射上限从"主循环节奏"还给了
线速（~820→理论 600+ 帧/s 满帧），剩余损耗是 5M 饱和下的 uart/RS485 线路错误
（67/12s）与 CRC（34/12s）累积——再往上只能动硬件。硬件改版移除 RS485 收发器，
改用 USB 全速 CDC（PA11/PA12，设备模式虚拟串口），TAB5 作为 USB 主机。

## 设计

### 时钟 160 → 192 MHz（用户决策）

USB 需要精确 48 MHz，160 MHz（HEXT×20）无整数分频可行。采用官方 vcp 例程
同款组合：`CRM_PLL_SOURCE_HEXT_DIV + CRM_PLL_MULT_48`（8M/2×48），
`crm_usb_clock_source_select(PLL) + CRM_USB_DIV_4` = 48 MHz。联动三处：

- TMR11 预分频 159→**191**（定时器时钟 2×APB2 = 192 MHz，1 µs tick 不变；
  probe_time.c 原注释即声明该硬编码依赖）。
- APB 80→96 MHz 后原 SPI 分频超传感器规格：SPI1(ADXL) DIV_8→**DIV_16**
  （6 MHz ≤ ADXL357 10 MHz 上限）、SPI2(ICM) DIV_4→**DIV_8**（12 MHz ≤
  ICM42688 20 MHz 上限）。读耗时 +~50%（ICM 512B 205→341 µs、ADXL 288B
  230→384 µs），仍远低于各自读取窗口（4 ms / 2 ms 节流），CPU 提速 20% 补偿
  成帧/调度路径。
- flash 等待周期：官方 192 MHz 例程与原 160 MHz 工程均不设置，维持厂商
  验证配置。

### USB 栈引入（零修改）

`AT32F413_Firmware_Library_V2.1.7` 的 `middlewares/usbd_drivers`
（usbd_core/usbd_int/usbd_sdr + usb_std/usbd_sdr 头）与
`middlewares/usbd_class/cdc`（cdc_class/cdc_desc）原样复制；不定义
`USB_VIRTUAL_COMPORT`（无 UART 桥接依赖）。`project/inc/usb_conf.h` 改自
vcp_loopback 例程：端点缓冲布局照抄（EPT1_TX 0xC0 / EPT1_RX 0x100 /
EPT2_TX 0x140），去掉 at32f413_board 依赖。关键库行为：
**`usbd_ept_send` 非 EP0 端点每次只发送一个 ≤64B 包、不自动续包**，
分包责任在 glue 层。

### probe_usb.c（链路 glue，~180 行）

对协议层暴露与原 probe_uart 完全同形的 API（init/read/send/poll/
error_count/tx_busy + isr，无 set_rate）：

- **TX 分包**：send() 把整帧拷入内部 1024B 暂存，立即发首个 64B 包
  （`usb_vcp_send_data` 同步写 PMA，上包未完返回 ERROR→报忙）；poll() 观察厂商
  全局 `cdc_struct.g_tx_completed`（IN 完成中断置位）逐包推进，**末包 TC 观测
  到才解除 tx_busy**——与 UART"线静默才闲"语义一致，window_service 的连发
  忙等 `while (tx_busy) poll()` 无需改动。
- **RX**：poll() 循环 `usb_vcp_get_rxdata`（每调用一个 ≤64B 包，取完自动重臂
  OUT 端点）灌入 1024B 环；OUT 包在未重臂时 NAK，无硬件回绕风险，环满丢新计数。
- **超时弃帧**：deadline 到即放弃（计数），PMA 中残留尾包由 TC 握手自然收敛，
  不污染下一帧。
- **初始化**：USB 外设时钟 + `USBFS_L_CAN1_RX0_IRQn` 抢占优先级 1（低于故障 0，
  高于时基 2/传感 3/DMA 4——连发循环每包靠 IN 完成推进，USB 延迟即线延迟）+
  `usbd_core_init` + `usbd_connect`。`usb_delay_ms/us` 用 probe_time 忙等。
  PA11/PA12 无需 GPIO 配置（USB 外设接管）。

### 协议层（probe_proto.c，最小 diff）

帧协议不变，TAB5 现固件零改动可跑（CDC 枚举为虚拟串口）。调用点
probe_uart_*→probe_usb_*（6 处）；**SET_LINK 对三档 legacy 速率改"接受即回显"
空操作**（原 BUSY 门/静默切换/3 秒回退机制删除），HELLO 能力位维持 0x0007；
主机侧 SET_LINK 5M 的既有上电序列照常完成。

### RS485 移除（用户决策：硬件已拆）

删 probe_uart.c/.h、probe_link_check.c/.h（RTT 触发的 5M 线缆质量工具）、
wk_usart1_init/RS485 引脚宏与 GPIO、DMA1 CH5/6（probe_dma 与 flexible 配置）、
validation 模式的 link_check 挂钩。

## 验证

- 全量 Unicorn 回归 **15 项 PASS**（irq×3 模式、board_check、calib、attitude、
  led、frame、fifo、validation、spi、**usb（新增）**、proto、acquisition、time、
  agm）+ pc_host 单测 6 PASS。
- 新 `test_probe_usb.c`（Unicorn，链接期打桩厂商 vcp 助手）：1024B 帧→16×64B
  分包时序与字节序、tx_busy 全序列（忙拒发/无 TC 不推进/末包 TC 才闲）、
  RX 环顺序与边界、过载丢新、超时弃帧后残留尾包收敛、init 调用拓扑与向量号。
- test_probe_proto fake 更名为 probe_usb_*，SET_LINK 断言改为"回显 OK/非法
  速率 INVALID"。
- 四模式构建 0 错 0 警；生产 **Code=61336 RO=2328 RW=280 ZI=49944**
  （较 round-16 +6500 Code / +952 ZI；64KB SRAM 内余 ~15KB）。

### 顺手修复（存量缺陷，均先在 HEAD 复现）

- `verify_board_image.py` 自 round-13（ZI>32KB）与 round-12（EOPB0 守卫读 USD）
  起失效：RAM 映射 0x8000→0x10000，补 USD（0x1FFFF800）映射并预置 eopb0=0xFA
  （已配置态）；stack 探针函数被 armlink 按未引用剪除，水位检查改宿主侧实现。
- `run_probe_spi.py --acquisition` 冗余入口自 round-15 起链不过（nvic 符号），
  移除；采集回归以 `run_probe_acquisition.py` 为准。

## 实板 round-17（Windows PC 直连，CDC 枚举 COM 口）

链路层全绿：枚举免驱、`host_crc_errors=0`（RS485 5M 时代的线路 CRC/uart 损耗
彻底消失）。slow 档（250/200Hz）ADXL 99%、ICM 98%、姿态 187/s（优于 RS485
时代 167/s）。

| 4k/8k 场景 | ADXL | ICM | 姿态 |
|---|---|---|---|
| FETCH 预算 1600（pc_host 原值） | 3926/s（98.2%） | 3656/s（45.7%） | 89/s |
| FETCH 预算 16384（TAB5 实机值） | **3983/s（99.6%）** | 3514/s（43.9%） | 85/s |

### 根因：发射忙等 × 主机包调度，饿死 ICM FIFO

- 帧序列连续性分析（dump）：两传感器**帧间零缺口**，ICM 损失集中在 31 次段
  重启 ≈ 2.6 次/s × ~1700 样本（~210ms/次）——"溢出→复位重配+60ms 预热"的
  特征签名；紧凑排水下 GET_STATUS 实锤：**ICM ovf=25/10s，ADXL 全程 seg=1
  零故障**。
- 因果链：Windows usbser 对 bulk IN 的调度实测 ~1 包(64B)/ms → 发一帧 1KB
  （16 包）期间，window_service 的连发忙等阻塞主循环 **14-16ms** → ICM
  FIFO（2048B = 16ms）每发大帧必溢出；ADXL 帧小（2-3 包 ≈ 2-3ms）幸存。
  RS485 时代同样的忙等只阻塞 1.5-2ms/帧（纯线速），无害——USB 适配时未识别
  这一时序语义变化。
- 反向佐证：慢排水节拍（FETCH 间隔 ~50ms）复现出相反的失败（ADXL ovf=76、
  ICM 满血 seg=1）——长突发里谁的 FIFO 余量小谁死，与阻塞模型一致。
- 附带发现：1600→16384 预算对 ADXL 有实效（98.2→99.6%），对 ICM 无效
  （瓶颈在溢出不在排水）；pc_host 增加 `--budget` 参数。

### 修复（`probe_proto.c` window_service）

连发忙等从"纯等包"改为**每个包完成间隙服务采集与姿态**：

```c
while (probe_usb_tx_busy()) {
    probe_usb_poll();          /* 每包纯线缆时间 */
    probe_acquisition_poll();  /* 突发期间 FIFO 不断粮 */
    probe_attitude_poll();
}
```

快主机间隙 ~100µs、这台 ~1ms，足够维持 2/4ms 的读节流节奏。全量回归 PASS，
Rebuild 0 错 0 警 Code=61344（+8B）。

## 实板 round-18（修复后复测，PC，`--budget 16384`）

| 指标 | round-18 USB | round-16 RS485 最好值 |
|---|---|---|
| ADXL 交付 | 3983/s（**99.6%**） | 99.8% |
| ICM 交付 | 7895/s（**98.7%**） | 85.4% |
| 姿态 | **195/s**（名义 200） | 150-177/s（负载下 48-89） |
| 主机 CRC | **0** | 34/12s |

- ICM 边界 seq 94861 = 样本数-1：**全程单段零重启**（修复前 31 次重启）；
  uncertain 帧 216/3138（仅启动收敛期，修复前 100%）。
- 总帧率 ~660 帧/s；帧均 ICM ~30 样本/ADXL ~20 样本（读节流下多为单读成帧，
  合并按需自动参与——若主机更慢、队列回压，合并会自行加深，自适配）。
- ICM 剩余 1.3% ≈ 启动 60ms 预热丢弃 + 时间线收敛期（~0.9s），随测量时长
  摊薄；更长时间测量（≥30s）下比例应进一步下降。

结论：**USB CDC 链路在所有指标上达到或超过 RS485 基线，ICM 交付从 85.4%
提升到 98.7%，线路 CRC 损耗归零**。剩余待办：≥30s 长时稳定性观察、秒长核对
（RTT vs RTC）、TAB5 实机（ESP32-P4 USB host，另一仓库）接入后复测。

## 实板探索：ICM 16k/32k ODR 不可行（本轮架构下）

实测（ADXL 同步 4k，`--budget 16384`）：

| 配置 | ICM 接受率 | ICM ovf | 线上实际 | 判定 |
|---|---|---|---|---|
| 8k（本轮基线） | 98.7% | 0 | ~105KB/s | 可行 |
| 16k 紧凑排水 | 6.1% | 53/12s | 13KB/s | 不可行 |
| 16k 慢排水（主循环空闲） | 7.0% | 53/12s | 15KB/s | 不可行 |
| 32k 紧凑排水 | 1.0% | 57/12s | 4KB/s | 不可行 |

两个独立瓶颈，任一都足以否决：

1. **探头侧**：慢排水（每 50ms 一个窗口、主循环基本空闲）下依然溢出——排除
   协议/USB 因素后，剩余嫌疑为采集管线的节奏预算：12MHz SPI 单次 512B 读
   341µs、状态机每 poll 推进一步、≥16k 时姿态重采样升到 4/5 级半带
   （每批 ~470µs 浮点 MAC），叠起来超过 ICM FIFO 在 16k 的 8ms 余量
   （32k 仅 4ms）。ADXL 在所有实验中 99%+ 健康。
2. **主机侧**：16k 需线上 ~208KB/s（13B/样本）、32k ~416KB/s，本机 CDC
   IN 调度实测上限 ~110KB/s。

若未来确需 16k：固件路线存在（高 ODR 档旁路姿态重采样、FIR 转 int16 MAC、
忙等间隙多步推进状态机、SPI 提速受限于 APB 96MHz/ICM 20MHz 规格不可行），
且必须换更快主机（TAB5/ESP32-P4）。**当前设计以 ICM ≤8k 为链路上限**，与
ADXL 4k 的主测量通道定位一致。

## 时钟重选（round-19）：HICK 48M USB + 160MHz 核心，SPI 回到规格顶格

16k 探索暴露的探头侧瓶颈里有 SPI 读粒度一项（12MHz 下 512B 读 341µs）。
用户决策：USB 时钟改走 **HICK 48M**（ACC 以 HEXT 晶振连续校准，厂商例程
USB_CLK_HICK 路径），把 PLL 从 48MHz 整除约束中解放；主频选 **SPI 最优的
160MHz**——APB=80MHz 时 SPI2÷4=20MHz（ICM42688 上限）、SPI1÷8=10MHz
（ADXL357 上限），两路同时顶格，且这是 USB 改造前经过 16 轮实板验证的原生
配置。TMR11 预分频回 159。变更：wk_config（PLL HEXT×20、删 usbdiv）、
probe_spi/wk_config SPI 分频、probe_time 159、probe_usb_init 增 HICK+ACC
选择序列（usb 回归补 CRM/ACC fake 并断言调用拓扑）。

**实板结果（round-19 复测，30s/16384 预算）**：HICK 枚举稳定、USB CRC=0、
SPI 20MHz 零故障（spi/fmt/ovf 全 0，改板信号完整性没问题）、ADXL 100.4%
（95 样本/帧满槽合并）；但 **160MHz 的 CPU 不够用了**——ICM 全部帧
time_uncertain（估计器 30s 未收敛）、段边界 churn、姿态 30s 仅 10 个输出
（round-18 为 2340）、慢排水下队列恒满 qdrop≈生产速率。192→160MHz 的
-20% CPU 与 USB 发射/协议开销叠加后越过临界点，SPI 提速（读 341→205µs）
补偿不回来。**结论：HICK 的价值是解放 PLL，而非降频**。用户再推一步：**200MHz**
（器件上限，APB 100MHz 顶格；200/4≠48，只有 HICK USB 才可达）+ SPI
12.5/6.25MHz（DIV8/DIV16），TMR11 预分频 199。回归全 PASS，四模式
0 错 0 警 Code=61432。**实板 round-20 全绿（30s/16384）**：ADXL 4020/s（100.5%，边界连续、
单段）、**ICM 7993/s（99.9%，全程单段，uncertain 仅 222 帧收敛期）**、
**姿态 199/s（名义 200 满速）**、TIME_INFO=30/STATUS=60 节拍正常、CRC=0。
四项判据全过且均为历史最佳——**200MHz 确认为标准工作点**，flash 时序
无异常。最终时钟树：200MHz + HICK 48M USB + SPI 12.5/6.25MHz +
TMR11 预分频 199。

### 为什么 PC 的 CDC 只有 ~110KB/s（USB 明明是 12Mbps）

12Mbps 是线信令速率：FS bulk 每笔事务带令牌/握手开销，理论净荷上限
~1.216MB/s（19×64B/ms）。实际落在 ~110KB/s 是三重叠加：① Windows
usbser.sys 老式驱动栈对 bulk IN 的调度粒度粗（实测约 1 包/ms）；② 设备侧
F413 USB 无 bulk 双缓冲，一次只能武装一个 64B 包，等 TC 再下一包；③ 我们的
协议是 FETCH 请求-响应式，不是连续推流。三者相乘即 ~64-128KB/s 量级，
与实测吻合。这不是 USB 总线极限——Linux cdc_acm 或 WinUSB 类栈可达
600KB/s-1MB/s；TAB5（ESP32-P4 专用 host 驱动）实测后见分晓。

## 未上板事项（round-19 待复测）

- 重烧后先确认 HICK 路径枚举稳定（反复插拔数次），再 `pc_host --port COMx
  --seconds 12 --budget 16384` 复测 4k/8k，应与 round-18 持平（ADXL ~99.6%、
  ICM ~98.7%）。
- 16k 慢排水复测探头的 ovf/接受率（SPI 读 341→205µs 与 CPU 192→160MHz 的
  净效应）；本机 IN 带宽仍封顶 16k 线上需求，交付率不会满。
- 若 HICK 枚举不稳：回退 PLL 144MHz + USBDIV_3（CPU/SPI 各降 10%）。

## 历史包袱交接

仍需人工实板：ICM 其余五面、ADXL 完整六面标定、显式保存/复位/断电验证
（round-16 交接项不变）。
