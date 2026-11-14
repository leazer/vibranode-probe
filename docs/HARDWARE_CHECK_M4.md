# RS-485 上板验证指南（后移执行）

当前先按 [单板 RTT 验证](HARDWARE_CHECK_RTT.md) 验证探头；默认已改为模式 3。执行本文前须显式改为 `PROBE_HARDWARE_VALIDATION=0` 并 Rebuild，日志应为协议入口。

烧录生产镜像 `project/MDK_V5/objects/VibraNode.hex`（`PROBE_HARDWARE_VALIDATION=0`，显式选择生产模式后包含：双 FIFO 采集 + RS-485 协议 + 时间映射 + 灯语 + AGM 姿态引擎 + 校准/Flash 读取）。RTT 仅有启动与链路事件日志，没有 M3 的周期统计和按键命令；如需复测 M3 采集本身，另编 `PROBE_HARDWARE_VALIDATION=2` 镜像。

## 一、无 PC 对端的最小检查

1. 上电后 RTT 应出现：
   - `[I][MAIN] VibraNode started`
   - `[I][PROTO] RS485 protocol link ready; RX armed at 1 Mbps`
2. 除此之外应保持安静：无对端时协议层不主动发送，采集未启动（等 SET_CONFIG/START）。Flash 里无有效校准记录时静默使用默认系数（无日志属预期）。
3. 接 USB 后 LED 蓝常亮属预期（已枚举、session 未绑定，状态 0 = 蓝常亮）；未连 USB 时为白闪 1 Hz（状态 6，固件本地）。

## 二、有 USB-RS485 对端时的完整会话

PC 端运行 `tests/pc_host.py`（扮演 TAB5 角色，`pip install pyserial`）：

```powershell
# 5 Mbps 完整会话，默认配置（ADXL 4 kSPS / ICM 8 kSPS），持续 10 s
python tests/pc_host.py --port COM7 --seconds 10 --dump raw.bin

# 仅 1 Mbps 链路（默认配置超出 1 Mbps 容量，必须用低速档）
python tests/pc_host.py --port COM7 --work 1000000 --config slow --seconds 10

# 故障注入：每 7 帧破坏一次本端 CRC，每 11 次 RX 丢弃一段数据
python tests/pc_host.py --port COM7 --corrupt-crc 7 --drop 11 --seconds 10
```

判定要点：

- 发现：`session 0x???????? bound`；上电后约 1 s 内应答。**绑定后 LED 蓝灯 2 秒心跳（100 ms 亮 / 1900 ms 灭）**。
- 切速：`link confirmed at 5000000`；若失败自动回退并重新绑定。RTT 侧出现 `link switched to 5000000 bps`；3 s 无帧应有回退告警。
- START 后 **LED 绿灯 1 Hz 双脉冲（100/50/100/750 ms）**。
- 流速率：稳态 ADXL ≈ 4054 样本/s、ICM ≈ 8039 样本/s（M3 实测时钟偏差 ±1.5% 内均正常）。
- 时间映射（M5）：会话最初几帧可能带 `time_uncertain` 标志，收敛后应为 0；`TIME_INFO` 计数应持续增长（段首 + 约 1 s 周期）。若 10 s 会话结束时 `time_uncertain_frames` 仍等于总帧数，说明时间映射未收敛，需排查。
- 质量：故障注入开启后 `host_crc_errors` 应大于 0 且会话不中断。
- 停止：`STOP boundary: ADXL seq N, ICM seq M`，最终排空后两流序号不小于该边界；**STOP 后 LED 回到蓝灯 2 秒心跳**。

### 数据物理合理性（raw.bin）

当前 `--dump` 保存完整 STREAM 帧（帧头、类型、会话/帧序号、载荷、CRC），保留 ADXL/ICM 区分；旧版只拼接载荷的文件不兼容，需要重新采集。载荷内是32字节元数据及样本记录。ADXL每条9字节，各轴为大端20位、位于24位字段高20位；ICM每条13字节，六轴为小端int16，加1字节温度。

```powershell
python tests/pc_host.py --inspect raw.bin
```

分别报告 ADXL/ICM 样本数与 Z 均值（原始计数）。默认量程下，重力朝 Z 轴时 ADXL 约 ±52429 count、ICM约 ±2048 count。不能把两路混在一起按9字节步长读取，也不能用小端int16读取ADXL高位。

导出/解码已有软件测试；PC会话工具和真实链路仍待后续联调，不能把自检通过当作持续吞吐验收。

## 三、LED 状态对照（2026-10-11 定稿）

| 现象 | 含义 |
| --- | --- |
| 白闪 1 Hz（500/500 ms） | 未连 USB 主机（未枚举/挂起/拔线；固件本地状态 6，不上线） |
| 蓝常亮 | USB 已枚举、session 未绑定 |
| 蓝闪 2 秒心跳（100/1900 ms） | 已绑定、未测量 |
| 绿闪 1 Hz 双脉冲（100/50/100/750 ms） | 测量中、无故障 |
| 紫闪 5 Hz（100/100 ms） | CALIBRATE 窗口采集中 |
| 红闪 5 Hz（100/100 ms） | 测量中出现告警（如 ICM 通道故障、时间戳未收敛） |
| 红常亮 | 测量中主通道（ADXL/ICM 身份）故障 |

线上协议的状态值 0-5 与灯语 ID 不变；黄色已停用。物理故障注入：测量中拔掉 ADXL 一侧排线，数秒内应转红常亮；ICM 断开应为红闪 5 Hz。恢复接线后故障是否自愈取决于采集层状态机，以实测为准并记录。

## 四、当前镜像验不到的部分（勿误判为故障）

- AGM 姿态（M7）：ATTITUDE 0x20已在FETCH窗口输出，`pc_host.py`会统计并保存Q30四元数；所有ICM ODR统一为200Hz。
- 校准（M8）：CALIBRATE/SAVE_CAL载荷已冻结。读取：`python tests/pc_host.py --port COM13 --work 4000000 --read-cal`。完整ICM六面：`python tests/pc_host.py --port COM13 --work 4000000 --config slow --calibrate icm-accel`；先不加`--save-cal`检查RAM候选，确认后才显式加该参数写Flash。单面断点可加`--face 0..5`。ADXL用`adxl-accel`，陀螺用`icm-gyro`。交互等待期间脚本持续FETCH，不能另开串口工具占用COM口。
- FETCH 看门狗（50 ms）与 5 Mbps 回退路径依赖对端配合注入，pc_host 的 `--drop` 可粗验。

## 五、尚未验收的边界

5 Mbps 长时持续吞吐（默认组合净载荷 140 kB/s，理论窗口余量见探头设计第 10 节）需 ≥10 分钟实测；DIR 方向切换波形需逻辑分析仪确认 TC 后无截断；M5 锚点误差、姿态质量、Flash 掉电保护等按各里程碑文档的上板项执行。以上未完成前不得宣称链路/功能通过验收。
