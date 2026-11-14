# AGM 姿态算法库（vendored）

来源：TDK `smartmotion-dkexamples-atmel-cm4-fpu-2.0.9`（GCC CM4 FPU 预编译包），复制进本仓库使固件仓库自包含；工作区 `references/ICM-42688/` 下的原始包仅为参考文档，构建不依赖它。

| 文件 | 来源 | 说明 |
|---|---|---|
| `lib/libInvnAlgoAGM.a` | `prebuilt/lib/libInvnAlgoAGM.a` | 预编译姿态算法（87 KB），未修改 |
| `include/invn_algo_agm.h` | `sources/Invn/LibAGM/invn_algo_agm.h` | 算法接口（输入 20 位满量程编码，±FSR = 1<<19） |
| `include/invn/common/invn_types.h` | `sources/Invn/LibAGM/invn/common/invn_types.h` | 公共类型，随头文件依赖复制 |
| `LICENSE.txt` | 包根 `license.txt` | TDK/InvenSense 软件许可协议，随库分发要求保留 |

调用约定见 `project/src/probe_attitude.c` 与 M7 里程碑文档；官方测试向量在 `tests/agm/out_invn_agm_1.h`（仅测试用，不参与固件链接）。升级库版本时整目录替换并核对新头文件依赖与链接符号。
