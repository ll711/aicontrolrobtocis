# 两棵决策树 + 鼠标路线控制

现在在 `DigitalTwin` 软件中画路线并上传，小车上的 C++ 执行两棵树。修改路线不需要重新编译固件。

```text
DigitalTwin 鼠标路线 → WiFi/TCP → StampC3_Template/Controller.cpp
  → 编码器里程计（毫米转换为米）
  → Tree1PathController + tree1_model.hpp：动作、顺时针目标转角
  → Tree2MotorController + Tree2Model.h：左右轮 PWM
  → Robot_c::setMotorPWM()
```

## 鼠标使用步骤

1. 先编译、上传本项目 StampC3 固件。上一级目录的 `platformio.ini` 已启用 C++17，默认上传串口是 **COM6**，按实际设备修改。
2. 用 Processing 4 打开上一级 `DigitalTwin/DigitalTwin.pde` 并运行。如果提示缺少 `processing.net`，在 Processing 的库管理器中安装 Network 库。
3. 电脑连接小车 WiFi **myAP**（密码 **myPassword**，均来自 `StampC3_Template.ino`）。软件自动连接 **192.168.4.1:80**；这里是 TCP 指令接口，不是浏览器网页。
4. 将小车放在你定义的原点，车头朝画面 **+X（右方）**，静止时点 **Reset pose**。界面中的橙色小车、橙色轨迹来自真实编码器定位。
5. 用 **Points** 逐点画折线，或选 **Freehand** 按住左键自由绘制。蓝色是草稿，第一点为绿色，终点为红色。通常从原点附近开始画。
6. 点 **Upload route**：软件先停车，再逐点上传并等待确认。上传完成后路线变绿色，**START** 才可用。
7. 点 **START** 执行。随时点 **STOP** 或按空格停车。想改路线时先停车、修改、重新上传，再启动。

| 操作 | 作用 |
|---|---|
| 左键单击（Points） | 添加路线点 |
| 左键拖动已有点（Points） | 移动路线点 |
| 左键按住拖动（Freehand） | 自由画线，松开后简化为折线 |
| 右键点击已有点 | 删除该点 |
| Undo / Z | 撤销一次编辑 |
| Clear | 清空草稿 |
| 鼠标滚轮 | 以鼠标所在位置缩放 |
| 鼠标中键拖动 / 方向键 | 平移地图 |
| H | 将视图移回原点 |
| T / G | 跟踪小车 / 显示遥测图 |
| STOP / 空格 | 请求实车停车并等待确认 |

离线时可以画路线；实车执行需要连接小车。运行中可直接按 Upload route 停车并重新上传；绘图编辑本身要求小车已停止。停止和上传均保留当前定位；只有 Reset pose 会归零。STOP 会重置路线进度，不是保留进度的暂停。路线自然执行结束后，需重新上传或 Reset pose，再明确启动。

默认每条路线最多 **256 个控制点、总长 4 米**，单点 X/Y 坐标范围为 **±10 米**。自由绘制会简化并显示最终蓝线，应按这条显示的路线执行。超限会提示，不能上传。路线在车端按训练时的 **1 mm** 间距重采样，固定容量 4096 点，约 96 KiB；控制器保持全局存储。

界面默认是 **Simulation** 双树仿真，灰色小车已经接入同一组模型权重。离线用 **Load into sim → START**；控制实车前选择 **Physical TCP**，再 **Upload route → START**。紫色为仿真轨迹、橙色为实车遥测。操作和仿真测试结果见 `../DigitalTwin/README_SIMULATION.md`。

## 模型和坐标

- **Tree1**：直接使用 `first_tree/cpp/tree1_model.hpp` 和 `tree1_config.hpp` 的已训练 River HAT，权重和训练配置不变。车端只推理，不在线训练。
- **Tree2**：2026-10-19 使用 `second_decision_tree/main.py` 对现有 `tree2_data.csv` 重新训练并覆盖本目录的 `Tree2Model.h`。输入特征改为顺时针误差、运动方向、上一周期右减左 PWM，并用左右镜像推理约束消除直行偏置。训练数据保持原来的 6144 条合成样本；报告见本目录 `Tree2TrainingReport.json`。
- 画布和编码器遥测内部是毫米，上传和 Tree1 是米；定位 yaw 为逆时针正，Tree1 输出给 Tree2 的转角为顺时针正。
- Tree2 预测 `right_pwm - left_pwm`，结合前进基准 45、倒车基准 -35 得到两轮 PWM。转角/终点距离会使基准减速；左右输出限幅到 [-100,100]，保持差分的共同余量。上一周期值来自上次成功发送的 PWM。
- STOP 直接输出零；ROTATE 使用单独的原地转向，左右相反且不超过 35 PWM。Tree2 没有训练 signal=0，不能用 signal 单独区分旋转和停车。
- 路线跟踪默认采用连续进度保护，限制交叉点、回到起点附近时跳到后段的情况。测试时可显式选择 `ProgressMode::SourceCompatible` 与原始 Tree1 几何实现逐项对照。此保护不改变模型权重。

`OdometryModel.h` 当前默认轮半径 16 mm、轮距 90 mm、每圈 358.3 编码器计数，应按实车校准。原始 Tree1 训练动力学用的是其他示例参数。

Tree2 当前是 **24 个合成会话、6144 条样本**，不是实车训练数据。按相同会话划分，验证 PWM 差值 MAE 从 **6.6561 降至 4.1913（降低 37.03%）**。前周期两轮 PWM 相等时，零转角预测差值为零；±0.1 rad 范围的教师规则最大误差为 0.001852 PWM，8194 个全角度检查中没有反向转弯预测。训练后仍需按实车校准；路线控制没有避障功能。镜像约束假设左右电机对称，换成不对称的实测数据时应重新评估这个假设。

## 指令、停止和重连

`RouteCommandParser.cpp` 接收按行分隔的 ASCII 指令，支持 TCP 拆包、合包和 CRLF：

```text
STOP
RESET_POSE
ROUTE_BEGIN 3
ROUTE_POINT 0.000000 0.000000
ROUTE_POINT 0.200000 0.000000
ROUTE_POINT 0.200000 0.150000
ROUTE_END
START
PING
```

每条指令有 ACK/ERR；BEGIN、POINT、END 的 ACK 带点数，软件逐条校验。完整路线确认前不能启动。上传失败、格式错误、断线或超时会停车并要求重新上传。软件每 500 ms 发送 PING，小车 **3 秒未收到有效指令**会请求停车；软件超过 2.5 秒无应答也禁用启动并断开。网络发送采用非阻塞方式，避免慢客户端拖延控制循环。

编码器读取、模型输入或电机写入失败会锁定停车并撤销路线启动资格。I2C 故障后仍会重试零 PWM；总线失效时不能保证电机收到请求。首次软件连接接管控制后，硬件按钮不再启动路线；未连接过软件时，按钮仍可运行 `AIPath.h` 的离线示例。

遥测保留 14 列 CSV：时间、x/y 毫米、逆时针 yaw、左右 PWM、左右编码器、5 路地面传感器、启动状态。新增 STATUS 行提供运行状态、动作、进度和错误，不影响原 CSV 解析。

## 文件与验证

| 文件 | 职责 |
|---|---|
| `Controller.cpp/.h` | 两棵树串接、控制周期、停止与遥测 |
| `Tree1PathController.cpp/.h` | 固定内存路线重采样、进度、Tree1 推理 |
| `Tree2MotorController.cpp/.h` | Tree2 差值转左右 PWM，旋转与停止 |
| `RouteCommandParser.cpp/.h` | 有界路线接收及指令解析 |
| `RobotWifiAP.cpp/.h` | TCP 连接和非阻塞收发 |
| `../DigitalTwin/RouteEditor.pde` | 鼠标编辑、上传确认、启动和停止 |
| `AIPath.h` | 无软件连接时的硬件按钮示例路线 |

在上一级项目目录执行：

```powershell
cmake -S . -B build-host
cmake --build build-host --config Release
ctest --test-dir build-host -C Release --output-on-failure
pio run -e stampc3
```

宿主测试使用旁边的 `first_tree` 作为独立原模型参考。若它在其他目录，配置时增加 `-DTREE1_SOURCE_DIR=实际路径`。固件输出是 `.pio/build/stampc3/firmware.bin`。

2026-10-19 验证结果：

- StampC3 固件编译成功：RAM 142620 / 327680 字节（43.5%），Flash 803426 / 1310720 字节（61.3%）。
- 3 组 C++ 测试全部通过，覆盖原始 Tree1 对照、闭合/交叉路线进度、双树电机输出、停止/故障、上传解析和控制周期。
- Tree2 的 5 项 Python 测试全部通过；实际部署头文件 25384 组 Python/C++ 对照通过，最大误差 0.000013882 PWM。
- `tests/run_processing_logic_checks.py` 执行真实 RouteEditor/TCPTelemetry 源码，通过 76 项无窗口逻辑检查，包含鼠标操作、毫米/米转换、上传 ACK、启动、停止、断线及超时。

后续已从本机下载的安装包解出 Processing 4.5.7，完整草图编译并打开实际窗口；双树已在课程 PDE 运动模型中完成 8 条路线测试（严格终点条件 7/8）。Java/C++ 另有 11070 组对照，界面逻辑检查扩展至 86 项。详见仿真说明及上一级 `simulation-results/report.json`。固件未烧录，实车运行尚未验证。

## 更新 Tree2

在 `second_decision_tree` 中运行现有导出器即可更新部署头文件：

```powershell
.\.venv\Scripts\python.exe main.py --csv tree2_data.csv --output "..\robotics_system_all_AI\robotics-system\RoboticsScienceAndSystems-main\StampC3_Template\Tree2Model.h" --report "..\robotics_system_all_AI\robotics-system\RoboticsScienceAndSystems-main\StampC3_Template\Tree2TrainingReport.json" --training-note "Synthetic teacher data; not measured robot performance." --verify-cpp
```

真实数据沿用 `session_id,alpha_rad,signal,pwm_l_prev,pwm_r_prev,pwm_l_target,pwm_r_target` 列和至少两个独立会话。更新模型后重新编译、验证，再上传固件。
