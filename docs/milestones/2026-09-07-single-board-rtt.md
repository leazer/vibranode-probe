# 单板 RTT 验证入口与实现修正

用户2026-09-07要求：先修复审查发现的问题，增加单板RTT验证，RS-485与TAB5通信联调后移。基线 `3b8ab36`，分支 `codex/probe-foundation`。

## 实现

- 默认模式3新增 `probe_board_check`，固定单实例上下文拥有配置、报告、校准候选与采集窗口。主循环独占双FIFO/AGM/LED/RTT，不与协议消费者同时运行；保留模式0/1/2供后续协议与旧诊断。
- 自动默认4k/8k采集；RTT启停/异步重启/四档切换/原始队列丢样注入，按秒输出状态、时间收敛、实测周期、姿态与错误计数。每轮最多消费每传感器一个批次，校准也有界处理，不堆放原始数据大数组。
- LED12秒巡检结束恢复状态；ADXL默认量程六面及ICM静止陀螺窗口，检查连续性、方向/运动/超时，成功结果仅保留RAM候选，停止后明确 `w` 才写Flash，`v`重载。两类校准更新合并，未完成/拒绝窗口不覆盖候选。
- 复位汇编在首个C调用前填栈；轮询测量采集/姿态/完整单轮最大耗时，RTT扫描栈水位。不声称这些观测值已证明WCET或全路径栈安全。

## 修复

- mode2的EXINT原先进入M2诊断器；现在只有mode1走诊断器，0/2/3均传到采集层。
- 姿态原先每批仅处理16包，512 B/32包批次被静默截断。scratch容量按512 B推导，借用批次到处理结束才释放；禁用姿态时仍排空旁路，避免伪丢批。初始化失败前先清active。
- 校准偏置统一为名义g单位，应用公式 `(raw/nominal - bias_g)*scale`；原测试复制了旧错误单位，已改独立期望并复现修复前失败。
- Flash v2增加generation；代次排序不受重启uptime归零影响，created_ms只用于记录时间。v1记录不兼容，会回退默认需重标。程序链接上限改为0x0803F000，明确保留末两个2 KB扇区。
- PC混合dump改存完整帧，`--inspect`可离线区分两路、按正确端序/20位符号解析。旧载荷拼接文件不兼容，需重新采集。修正485指南误写的AGM数据源ADXL→ICM。

## 验证证据

- `tests/run_probe_spi.py`：11个组件通过（原10个加board_check）。board_check含六面/陀螺成功、错误朝向/运动/超时拒绝、异步停止、运行中禁写、重复保存、合并候选与LED结束恢复。
- `--irq 0/1/2/3`：四模式运行实际IRQ入口，修复前mode3失败，修复后均通过。
- calib含重启后小created_ms仍为最新记录、CRC损坏回退、每个写入字前注入失败后保留前一条完整记录。
- `tests/run_probe_time.py` 与 `tests/run_probe_agm.py` 官方全向量通过；RTT项目集成和日志宏四组通过。
- `tests/test_pc_host.py` 两项（混合帧、正负极值、非法长度）及pc_host自检通过。真实会话/吞吐未验收。
- `tests/verify_build_modes.py` 四模式完整Rebuild均0错误0警告；最后留下原项目默认模式3镜像。
- `tests/verify_board_image.py` 在当前实际AXF上执行Reset_Handler至SystemInit之前，验证整栈填充，再调用实际水位函数验证覆盖检测；检查Flash分配段不侵占标定区。
- 独立只读审查后，已补齐陀螺运动拒绝与逐字Flash中断测试；无未处理阻断发现。

| 模式 | Code | RO | RW | ZI |
|---|---:|---:|---:|---:|
| 0 协议生产 | 44328 | 2324 | 16 | 28824 |
| 1 M2诊断 | 18208 | 1756 | 16 | 6256 |
| 2 M3采集 | 21696 | 2052 | 16 | 15248 |
| 3 单板RTT | 47716 | 3900 | 16 | 23896 |

ZI包含2 KiB栈；模式3未用RAM为8856 B（约8.7 KiB），模式0为3928 B（约3.8 KiB），不能把模式3余量当作生产模式余量。姿态scratch增加256 B；单板上下文不进入生产镜像的存活数据。

默认HEX：`project/MDK_V5/objects/VibraNode.hex`，SHA256 `B94D83F3EC4A6A08B021BA1244A72EC42CE831E37BA98D229A3C7251F1B92483`。

测试环境依赖 `pyelftools==0.33`、`unicorn==2.1.4`、Keil ARMCC5，当前安装于忽略目录 `tests/build/deps`，执行Python测试前设置 `PYTHONPATH` 指向该目录。构建与测试产物不入Git。

## 下一步

按 [HARDWARE_CHECK_RTT.md](../HARDWARE_CHECK_RTT.md) 烧录/收集单板日志。尚未烧录、没有本轮实板结果；真实锚点精度、AGM真实振动品质、噪声阈值、WCET/栈、Flash擦写/掉电须上板，不能由仿真代替。ICM加速度六面、非默认量程校准和线上ATTITUDE/CALIBRATE/SAVE_CAL仍待后续。单板问题收敛后，再做485/PC/TAB5通信与吞吐，随后TAB5测量内核。
