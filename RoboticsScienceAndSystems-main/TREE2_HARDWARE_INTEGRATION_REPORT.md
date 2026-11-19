# Tree2 实体小车固件集成报告

2026-11-19。默认 `stampc3` 固件已接入 `measured-forward-v1`，最终应用镜像已生成。目标 ELF 中查到新模型及其 144 字节叶表，回滚 ELF 查到旧模型。桌面对照实际执行了固件使用的控制代码，并核对写到电机接口边界的整数值。PlatformIO 串口枚举返回空列表，**烧录、离地转轮、低速地面试验和板上计时均待实板接入**。[新固件](artifacts/tree2-v1/stampc3/firmware.bin) · [回滚固件](artifacts/tree2-v1/stampc3_legacy/firmware.bin)

## 推理位置和电机链路

工程入口为 `StampC3_Template/StampC3_Template.ino`，PlatformIO 将该目录配置为 `src_dir`。构建输出确认目标为 Espressif ESP32-C3，板型配置 `esp32-c3-devkitm-1`、160 MHz、Arduino 框架、GNU C++17。两棵树编译进 StampC3 应用。Processing 在 Simulation 模式运行对应的 Java 版本，在 Physical TCP 模式向 StampC3 发送路线和控制命令。底盘中间件接收轮命令并执行电机驱动，本工程可核实到 I²C 协议及主控写入端，下位机电机驱动器的电气极性要在台架核对。[固件入口](StampC3_Template/StampC3_Template.ino) · [构建配置](platformio.ini)

| 运行步骤 | 实际代码与约定 |
|---|---|
| 1. 编码器与 Tree1 | `Controller_c::update` 每 10 ms 采集。首帧建立编码器基线。每 50 ms 调用 `runDecisionTrees`，把里程计位置 mm 转为 m，传入逆时针为正的 yaw，单位 rad |
| 2. Tree1 交付动作和角差 | `Tree1PathController::step` 交付原始动作枚举、信号、顺时针为正的 `delta_theta` 及几何信息。STOP 和 ROTATE 信号同为 0，固件按枚举分支。运动角差规约到 (−π, π] |
| 3. 新 Tree2 | FORWARD 以角差匹配 C++ 表中的 CART 叶，读取 PWM 差值，随后执行叶间插值和小角度保护。REVERSE 沿用旧教师的增益、上限及记忆系数，历史差值仅在与当前转向同向时参与。两者都产生右减左 PWM |
| 4. 分配左右轮 | 结合前进 +45、倒车 −35 的基准、角度和距离衰减，以及新版前进速度配合规则。将差速限制到 ±2(100−\|base\|)，再计算 `L = lround(base − Δ/2)`、`R = lround(base + Δ/2)`。每轮为 [−100, +100] 的有符号整数 |
| 5. 写入底盘 | `Robot_c::setMotorPWM(L,R)` 构造两个 int16，索引 0 为左、1 为右，写到 I²C 地址 0x08 的寄存器 0x06。事务成功后才更新命令缓存，供下一周期使用。正值请求轮向前、负值请求轮向后。当前协议给出的数值单位是 PWM 指令 |

STOP 直接写零。ROTATE 调用原来的 `turn = clamp(45 × eCW, −35, +35)`，左右分别为 `turn` 与 `−turn`。测得的左转正差速和右转负差速与里程计 `(右轮位移 − 左轮位移) / 轮距` 的航向定义一致。闭环时每轮的实际速度、停转和通道接线仍要独立观测。[控制周期](StampC3_Template/Controller.cpp) · [差速分配](StampC3_Template/Tree2MotorController.cpp) · [I²C 写入](StampC3_Template/Robot.cpp) · [协议定义](StampC3_Template/i2c_datatypes.h)

## 版本、稳定机制与故障行为

- **测得的覆盖范围。** 前进左右弧线的剩余角幅度为 0.153°～90.179°。新树不使用前一周期差速，前进的 75° 翻号触发条件因此被移除。超出叶中心范围时保持边缘叶值。倒车保留旧教师的 90 PWM/rad、140 PWM 上限及 0.65 历史权重，同时剔除反向历史并限制其幅度。该分支属于工程规则，倒车数据仍需采集。
- **外围控制。** 插值消除常数叶边界的阶跃，零点锚定、±0.36° 死区和带外增益限制用于抑制零点附近反复换向。前进差速不足以实现名义转向时同比降低平均 PWM。以上规则的贡献可在原始 CART、插值、最终浮点和最终整数四层结果中分别查看。[训练与控制依据](docs/TREE2_RETRAINING_REPORT.md)
- **停车路径。** 主机测试运行真实的 `Controller.cpp` 和里程计，电机写口及网络由测试替身记录。通过了 STOP、几何终点、远程断连、3,000 ms 心跳失效、编码器失败、无效角度或距离、超范围历史 PWM、电机或网络写失败的检查。远程失联撤销启动资格。电机通信故障会请求零 PWM 并在后续采集周期重试。I²C 中断时是否已物理停车仍须靠实板和独立断动力措施验证。[控制周期测试](tests/controller_cycle_tests.cpp) · [输入及边界测试](tests/tree_models_tests.cpp)

## 目标固件编译、空间与时间

实际使用 Espressif32 平台 6.8.1、Arduino 框架包 3.20017.241212、RISC-V 工具链 8.4.0+2021r2-patch5。在同一次 `pio run -e stampc3 -e stampc3_legacy` 中，两版都成功链接并生成 ESP32-C3 应用镜像。

| Release 构建 | 静态 RAM / 327,680 B | 程序 Flash / 1,310,720 B | 应用 .bin 文件大小 |
|---|---:|---:|---:|
| 新版 `stampc3` | 142,620 B，43.5% | 802,302 B，61.2% | 825,328 B |
| 旧版 `stampc3_legacy` | 142,620 B，43.5% | 803,426 B，61.3% | 826,432 B |

新版程序 Flash 少 1,124 B。用目标工具链的 `riscv32-esp-elf-nm -S -C` 读取最终 ELF，默认镜像包含 `tree2_measured::predict` 和大小 0x90 的 `tree2_measured::LEAVES`，回滚镜像包含 `Tree2Model::rawPredict`。每个新叶存三个 float，分别为分裂上界、训练角中心和 PWM 均值。查找最多比较四次，随后至多一次相邻叶插值，无堆分配。

在 Windows x64 的 MSVC Release 构建中，对完整前进电机适配器遍历角度，五轮各调用 100 万次，包含角度规约、余弦减速、推理、混合和取整。每轮均耗时的中位数为新版 **21.20 ns/次**、旧版 **16.53 ns/次**。控制周期测试另行核实每 50 ms 产生一次决策，采集间隔为 10 ms。板上最坏推理耗时、WiFi/I²C 负载和实际周期抖动需要在运行中的 C3 测量。

本次产物 SHA-256 如下，二进制可能随工具链和构建元数据变化。

| 应用镜像 | SHA-256 |
|---|---|
| `.pio/build/stampc3/firmware.bin` | `5898864a136522c5f4c0a7ff4462a229a1da8834432825f029d61114cf6ec834` |
| `.pio/build/stampc3_legacy/firmware.bin` | `33c7c8e7ba194a50d875cb5bac3bd737027ea576479a92426f5261ee81966ad2` |

## 实际执行的一致性测试

| 检查 | 实际结果 |
|---|---|
| 新 Python 与固件同源 C++17 | 留出样本、±75° 全历史轮差、全角度网格、分裂相邻 float、叶中心及死区边界，共 77,796 组。原始 CART 叶值按 float32 比较完全一致，最终差速请求最大绝对误差 0.000006871 PWM |
| 旧 Python 与归档 C++ | 用已核对 SHA-256 的旧合成 CSV 和原训练流程复建，再对 6,144 条合成输入、11,610 条留出输入和 4,000 组随机输入比较，共 21,754 组。最大绝对误差 0.000017564 PWM |
| 两种 C++ 构建的轮输出 | 各 77,796 组检查实际适配器输出与独立混合、限幅、取整计算，整数轮命令不一致数均为 0。两个构建均使用各自选定的模型 |
| 实际小车控制入口 | 在 ±75° 的路径和编码器基线夹具中，真实 `Controller_c::runDecisionTrees` 先运行 Tree1，再运行选定版本，测试核对传给 `setMotorPWM` 的左右值。新版和回滚版均通过 |
| C++ 与 Processing 4.5.7 | 完整草图重新编译。两版各对照 11,070 个输入，动作、进度、目标索引、终点标志和整数 PWM 均无差异，连续几何量最大误差 1.78×10⁻¹⁵ |
| 现有软件回归 | 两版各通过 3/3 组 CTest。每版含 223,878 项树与几何断言、233 项控制周期断言和 402 项路线协议断言。86 项 Processing 界面及协议逻辑检查通过 |
| 八条理想动力学路线 | 两版各 7/8。新版闭合方形在 5.011 mm 处被 Tree1 提前停车，旧版交叉路线在 5.060 mm 处被提前停车。两版的路线汇总均保留失败项 |

旧版的强制镜像和新版的实测叶值会产生不同预测。一致性测试分别用各自的参考值核对实现，误差比较则使用相同的实验记录。[逐角度回放结果](docs/tree2_evaluation.json) · [新版路线运行记录](artifacts/tree2-v1/stampc3/simulation.json) · [回滚版路线运行记录](artifacts/tree2-v1/stampc3_legacy/simulation.json)

补做特征和外围策略消融时，重新编译了新版与回滚版 StampC3 固件，A/C 两组的 16 条 Processing 轨迹与上表归档逐字节相同。该次 Windows Release 复跑中，两版的树模型与路线协议测试均通过，旧版 Java/C++ 的 11,070 例仍为零项离散差异。系统应用控制阻止两版控制周期测试和新版 C++ 对照程序启动，因此其完整通过记录取自上表的先前运行。当前串口枚举为空。[特征与闭环消融报告](docs/TREE2_FEATURE_COMPARISON.md)

## 复现、烧录状态与回滚

在项目根目录执行以下命令可编译固件及重新运行两版仿真。完整仿真脚本在任一路线未达到几何终点时返回 2，同时保留测试和报告。两版使用独立的输出目录。模型训练与 PWM 回放命令见[训练报告](docs/TREE2_RETRAINING_REPORT.md)。

```powershell
pio run -e stampc3 -e stampc3_legacy
$python = (Resolve-Path '..\..\second_decision_tree\.venv\Scripts\python.exe').Path
& $python -B tests/run_processing_simulation_checks.py --tree1-source ../../first_tree --cpp-build-dir build-host/tree2-v1 --results-dir build-host/tree2-v1/verification
& $python -B tests/run_processing_simulation_checks.py --legacy --tree1-source ../../first_tree --cpp-build-dir build-host/tree2-legacy --results-dir build-host/tree2-legacy/verification
& ./build-host/tree2-v1/Release/tree2_replay.exe --benchmark
& ./build-host/tree2-legacy/Release/tree2_replay.exe --benchmark
```

本机 Windows 串口枚举为空，`pio device list --json-output` 返回 `[]`。`platformio.ini` 内的 COM6 是配置值。现场接入且确认端口、板型和底盘状态后，新版使用 `pio run -e stampc3 -t upload --upload-port <已确认端口>`。回滚使用 `pio run -e stampc3_legacy -t upload --upload-port <已确认端口>`。回滚环境设置 `TREE2_USE_LEGACY=1`，同时恢复旧差速和原平均 PWM 规则。Arduino 单独编译时可用同名编译宏。旧头文件和旧训练报告均已保留。[旧模型](StampC3_Template/Tree2Model.h)

### 下一次实车验收

1. 现场核对端口和目标板，固定底盘、两轮离地，先断开电机动力完成烧录。读到可用遥测后核对静止读数、STOP 和左右编码器通道，再接入限流的电机动力。
2. 从复位位姿、零历史和 8 mm 短路线开始。本机已单独验证首个前进决策，直线的预期 (L,R) 为 (18,18)，顺时针 5° 为 (22,14)，逆时针 5° 为 (14,22)。同长度的顺时针 75° 为 (8,−7)，逆时针 75° 为 (−8,10)，可在离地时核对内轮负指令的方向。这些是首个命令，后续命令随编码器位姿变化。分别记录左右轮实际方向、启转及停转延时。仍在离地条件下核对 STOP、断连和停止心跳后的零命令与物理停转。
3. 通过离地检查后，在受控区域以相同电源、轮胎和地面做新旧版低速 A/B。分别覆盖直行、±5° 反复过零、左右 75°、转向后回直和倒车。记录时间、模式、目标与实际角度、左右下发 PWM、编码器及独立航向观测。用相同容差和观察窗比较峰值超调、回摆次数、稳定时间、路线时间和提前停车，并测量 C3 控制周期及最坏耗时。若低占空指令不能启转，保留原始轮速与电流记录再做死区标定。

## 项目文件清单

| 范围 | 新增 | 修改 |
|---|---|---|
| 固件和板型选择 | `StampC3_Template/Tree2MeasuredModel.h`、`StampC3_Template/Tree2ControlPolicy.h` | `StampC3_Template/Tree2MotorController.cpp`、`StampC3_Template/Tree2MotorController.h`、`platformio.ini` |
| 训练、模型与评估 | `tools/tree2_data.py`、`tools/train_tree2.py`、`tools/evaluate_tree2.py`、`models/tree2_measured_v1.json` | `tools/export_processing_models.py` |
| Processing | — | `DigitalTwin/TreeModels.java`、`DigitalTwin/TwoTreeController.java` |
| 编译与回归测试 | `tests/tree2_replay.cpp` | `CMakeLists.txt`、`tests/controller_cycle_tests.cpp`、`tests/check_processing_parity.py`、`tests/run_processing_simulation_checks.py` |
| 报告和说明 | `TREE2_HARDWARE_INTEGRATION_REPORT.md`、`docs/TREE2_RETRAINING_REPORT.md`、`docs/tree2_training.json`、`docs/tree2_evaluation.json` | `StampC3_Template/README_AI.md`、`DigitalTwin/README_SIMULATION.md` |
| 后续特征与控制策略消融 | `tests/run_tree2_ablation.py`、`tests/Tree2AblationProbe.java`、`docs/TREE2_FEATURE_COMPARISON.md`、`docs/tree2_ablation.json` | `tools/export_processing_models.py`、`tests/run_processing_simulation_checks.py`、两份原有训练和集成报告 |

开始时 Git 已显示根 `README.md` 有修改，`controldata/`、`data/` 和已有 `docs/TECHNICAL_DESCRIPTION_ZH.md` 属于未跟踪内容。这些文件和原始实验记录保留原样。Tree1 的模型、配置、几何适配器、动作语义和停车容差沿用原工程。
