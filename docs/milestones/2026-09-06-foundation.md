# M1：板级基础与算法兼容性验证

日期：2026-09-06。独立仓库基线：`417643e`；开发分支：`codex/probe-foundation`。

## 已完成的软件工作

- Keil ARM Compiler 5.06 update 7 build 960 的原始工程基线构建：0 错误、0 警告。
- 两个传感器 CS 初始化为高，PA8 保持接收，RGB 初始熄灭；移除主循环阻塞延时和灯光演示，保留用户时钟树与 RTT。
- TMR11 以既定 160 MHz 定时器时钟除以 160，生成 1 μs tick；16 位回绕扩展到 64 位。唯一持久状态为组件上下文，读写高位使用短临界区，保留调用者 PRIMASK。
- 六路 DMA1 弹性映射与仲裁优先级完成初始化。通道保持关闭；未实现或验证完整 SPI/UART 异步事务和方向收尾。
- 实际 C 时间组件使用 ARMCC 编译，在 Cortex-M 指令模拟环境中通过初始化、ISR 尚未执行的回绕、重复读取、伪中断、读取期间回绕及 PRIMASK 保留测试。寄存器为替身，不代表真实外设已验收。
- TDK 原始 GCC `.a` 在独立 ARMCC 测试程序中成功链接：测试使用 `--cpu=Cortex-M4.fp --apcs=/hardfp --wchar32`。首次未设 `--wchar32` 时触发 L6242E，修正测试目标的类型属性后通过，没有忽略链接诊断或改动预编译库。
- 原始 `out_invn_agm_1.h` 全部 3000 行向量逐项核对四元数与加速度/陀螺仪/磁力计零偏，结果完全一致；另以产品默认六轴参数运行 1000 次，验证输出 mask 和四元数范数。
- 原有 RTT 工程集成检查通过，日志宏四组配置检查通过。

## 内存证据与实际范围

固件构建：Code=10026、RO-data=506、RW-data=16、ZI-data=2840 字节。此镜像尚未包含 AGM 和姿态滤波。

独立 AGM 测试镜像：Code=20152、RO-data=288000、RW-data=0、ZI-data=2016 字节，其中官方输入/期望向量占 288000 字节，算法库被链接的代码为 15556 字节。该测试镜像放在模拟器运行，不可直接烧入 256 KB Flash 的探头。静态 2016 字节不包括栈，不等于最终集成总 RAM。

测试输入使用官方原始 ±32 g/±4000 °/s 配置，仅用于与其期望结果对照；产品开放范围仍为设计文档批准的范围。

## 复现

依赖：Python、`unicorn==2.1.4`、`pyelftools==0.33`（兼容版本可验证后使用），以及 `C:/Keil_v5/ARM/ARMCC/Bin`。测试脚本从自身目录解析仓库路径。TDK 测试读取外层 `references/ICM-42688/smartmotion-dkexamples-atmel-cm4-fpu-2.0.9`；独立检出固件仓库时需提供该参考包，脚本会在依赖缺失时失败，不会跳过后报告成功。

```powershell
python tests/run_probe_time.py
python tests/run_probe_agm.py
./tests/verify_rtt_project_integration.ps1
./tests/verify_vibra_log_macros.ps1
```

本次 Python 依赖安装于外层 `output/probe-test-deps`，通过 `PYTHONPATH` 引用，未提交二进制依赖。构建和测试输出在忽略目录中；本文件保留可追溯的结果摘要。

## 尚未完成的硬件/集成验收

- 无实际烧录或逻辑分析仪测量；TMR11 真实时钟/跨回绕、GPIO 上电波形、SPI 10/20 MHz、USART 5 Mbps 与 DIR 边界均待上板。
- DMA 短事务、FIFO 驱动、协议、姿态重采样和校准尚未实现。此里程碑不是完整第一阶段验收完成。
- AGM 尚未链接进正式 Keil 工程。后续集成需统一项目 ABI（包含 wchar32 的全目标重编译）并复查第三方接口；本次只证明其可由 ARMCC 链接执行。
- 实际 CPU 最坏耗时、栈高水位、12 KiB 候选缓存余量与强振动效果待测，不用模拟器执行耗时代替板级时间。

下一里程碑：实现可测试的 SPI DMA 事务层和双传感器身份读取，再进行实板时序验证与 FIFO 采集。
