# M3 双 FIFO 采集

芯片配置与解析在 `middlewares/sensors`，采集状态/原始队列在 `probe_acquisition`，测试消费和命令在 `probe_capture_check`。没有继续扩展 M2 的 `probe_validation`。

实现：

- ADXL 全部已约定 ODR/量程的配置表生成、FIFO 组解析、所有轴的空读/X 标记检查；ICM 同步六轴配置、分银行 AAF 配置与回读、字节计数、大端 16 字节包、1 us 时间戳及段内回绕扩展。
- 双路独立 DMA、启动丢弃、中断提示与定时兜底、每次最多 512 字节、读取前后溢出检查；错误重建只影响出错设备，ID/配置错误锁定等待显式重启。
- 每路四槽原始批次队列。满时保留旧数据，丢弃新批次、计数并切段；无动态分配，批次借用仅限主循环。
- SPI 层直接在其持有的 TX 缓冲生成读取时钟，避免调用者构造 513 字节栈数组。
- ICM 使能后保持至少 60 ms，再开启 FIFO；停止请求也不能绕过该保护。保留 GYRO_CONFIG_STATIC2 的 0xA0、GYRO_CONFIG1 的 bit 4 等保留位复位值。

时间模型、正式协议 STOP/尾部排空、姿态和标定未在本轮完成。M3 测试消费者会释放批次，不是持久记录。高 ODR 可做采集验证，不能据此声明链路能力。

## 来源核对

- [ADI 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl356-357.pdf)：FIFO 组标记与待机复位行为。
- [TDK DS-000347 v1.6](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf)：包型、计数、时间、AAF 表与寄存器保留位。AAF 表项直接取手册，未用简单平方替代。
- 本地 no-OS 与 Icm426xxDriver_HL 代码；继续沿用上一里程碑保留的厂商声明。

## 验证

芯片配置/解析测试、SPI DMA 测试、M2 回归、M3 调度仿真均通过。覆盖包型/短包/无效值、队列满、读取期间溢出、独立传输失败、停止与重启、ICM 模 65536 时间扩展。仿真不能代替真实采样率、模拟滤波响应和电气测试。

Keil 链接调用图报告主调用链 784 字节（带未知项，不含完整异常嵌套）。将实际参与编译的 `project/MDK_V5/startup_at32f413.s` 栈预留改为 2 KiB；运行期栈高水位仍待上板测量。

硬件验证流程见 `docs/HARDWARE_CHECK_M3.md`。最终全量编译 0 错误、0 警告：Code 17540、RO 2052、RW 16、ZI 12672 字节（含 2 KiB 栈预留）。

HEX SHA256：`23DCBA213207FB3072FFDB2B94BA51F4EB2491621AB36433D4C30BF028F76EDB`。

## 2026-09-07 实板验证

用户烧录本镜像并回传完整 RTT 日志（约 289 s）。操作序列：上电默认配置连续采集约 193 s → `x` 诊断中止 → `s` 重新开始 → `2` 切换 ADXL 250 / ICM 200 SPS → `3`、`4` 单独验证 ICM 16/32 kSPS → 32 kSPS 下两次 `r` 重启。

结果：

- 全程 spi=0 ovr=0 fmt=0 qdrop=0 gap=0 seg=1 fault=0，queue_max=1，主循环 poll_gap_max_us 最大 569。默认档 9.0～73.0 s 间一次 RTT Viewer 断连重连，固件侧计数跨断口速率一致且无溢出/缺口，属上位机断连，非设备异常。
- 相对 MCU 时间轴的实测速率：ADXL 4 kSPS 实测 4054 样本/s（+1.35%）、250 SPS 实测 253/s（+1.3%）；ICM 8 kSPS 8039/s、16 kSPS 16080/s、32 kSPS 32159/s（均约 +0.5%）。两颗传感器偏差不同且各自在各 ODR 下一致，符合传感器内部时钟与 MCU 无关的预期，是时钟偏差不是丢样。
- 静止数据物理合理：ADXL Z≈49700（±10 g 约 +0.95 g），X/Y 约 ±0.05 g；ICM Z≈2070（±16 g 约 1.01 g），陀螺静止峰峰值约 ±60 LSB（约 ±0.9 dps @±500 dps）。
- 启动丢弃样本数与 60 ms 稳定等待策略一致：ADXL 15@250 Hz～238@4 kSPS；ICM 12@200 Hz～1927@32 kSPS。ICM 16 位时间戳段内多次回绕，fmt=0 表明相邻包间隔检查未报异常。
- ICM INT1 实测行为：8 kSPS 下按约 11 包水位触发（irq≈726/s）；16/32 kSPS 下主循环连续排空快于水位累积，水位中断不再 pacing（每次启动仅 +1 边沿），读取由周期兜底承担，FIFO 高水位分别约 9/18 包且无溢出。与中断+定时兜底结构一致，记为实测行为。
- 未执行项：`q` 队列满注入、`a/i/d/e/g` ODR/量程循环、翻面轴向检查、30 分钟级长时运行；`l` 为 M2 遗留按键，本模式忽略。`x` 仍为诊断中止，非正式协议 STOP。

据此，M3 本地双 FIFO 采集链路在默认、低速与 ICM 高速档通过首轮实板验证；统一时间映射、正式协议出口、姿态与标定仍属后续里程碑。
