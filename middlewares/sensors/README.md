# 传感器组件

当前包含 M2 快照接口与 M3 FIFO 配置/解析接口；采集调度在 project 的独立模块中。

## 层级及依赖

| 层 | 代码 | 责任 |
|---|---|---|
| 芯片操作 | `adxl357/`、`icm42688/` | 寄存器定义、SPI 命令编码、身份检查、寄存器快照解码；无 MCU、RTT、GPIO 或 DMA 依赖 |
| 板级传输 | `project/src/probe_spi.c` | SPI/DMA、CS、超时和错误收尾；持有异步收发缓冲 |
| 采集服务 | `project/src/probe_acquisition.c` | 初始化回读、双 FIFO 调度、段边界、原始批次队列；通过芯片接口及板级 SPI 工作 |
| M3 测试消费者 | `project/src/probe_capture_check.c` | RTT 命令、批次消费和统计，不直接操作芯片 |
| 诊断应用 | `project/src/probe_validation.c` | 检查顺序、等待与回读、失败重试、10 ms 快照调度、日志及测试灯光 |

这里仅共享 `sensor_register_setting` 数据结构，不引入通用设备框架或函数指针分发表。ADXL357 只有加速度输出，不伪造陀螺仪接口。两个组件目前无可写全局状态；诊断上下文与 DMA 上下文分别管理各自生命周期。

组件接收的快照是寄存器有效载荷，不含 SPI 命令占位字节。返回的配置数组为只读静态常量，调用方不得修改。芯片命令函数要求调用方传入有效寄存器地址；ICM 当前操作限 bank 0。

`*_diagnostic_settings()` 明确是本次上板验证使用的固定配置，不是正式采集默认配置接口。范围/ODR 配置与 FIFO 解码由各自芯片组件的独立 FIFO 接口实现；采集会话、排队及样本时间归属正式采集模块。生产路径不能调用诊断轮询作为采集入口。

## 参考代码来源

- ADI：外层 `references/ADXL357Z/drivers/no-OS_adxl355_adxl357.h/.c`。`adxl357_regs.h` 提取所需寄存器定义，保留完整版权许可。参考源包含其他型号及 no-OS 的 SPI/I2C、分配和延时依赖，未直接移植整套驱动。
- TDK：外层 `references/ICM-42688/smartmotion-dkexamples-atmel-cm4-fpu-2.0.9/sources/Invn/Drivers/Icm426xx/`。`Icm426xxDefs.h` 复制原有定义，仅清理行尾空格，保留版权；包装实现使用 `ICM42688P` 选择目标型号。参考 `Icm426xxTransport.c` 的同步调用约定，本次没有将同步接口伪装为异步 DMA 完成接口。
- 自有包装代码保留 M2 已实测寄存器序列和等待时间，使用本项目 SPI DMA 完成后的数据进行解析。没有引入厂商平台初始化、阻塞延时或动态分配。

M3 配置表包括分银行 AAF、FIFO、时间与中断，生成前检查配置合法性；实际执行与等待由采集服务负责。TDK AGM 仍处于独立验证阶段，未作为本次传感器组件重构的一部分接入运行固件。

## 板级约束

板级引脚定义唯一来源是 `at32f413_wk_config.h`。调用 CS、LED、RS485 DIR 时引用其 `*_GPIO_PORT`、`*_PIN`；不能在中间件或应用里重复写 GPIOA/GPIOB、GPIO_PINS_x。即使多个 LED 当前位于同一个端口，也分别引用各自的端口定义。

## 验证

`tests/run_probe_spi.py --validation` 将两个组件和诊断应用一起编译仿真，检查 M2 写寄存器顺序和值、ID、正负极值、长度边界、独立错误及重试。`tests/run_probe_spi.py` 验证底层 DMA 状态机，`--fifo` 验证芯片配置/解析，`--acquisition` 验证调度和错误路径。Keil 项目中的 Sensor components 组直接编译这里的两个实现。
