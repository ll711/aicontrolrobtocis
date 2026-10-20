# Processing 4 双树路线仿真

默认运行 **Simulation**。灰色小车由 Tree 1 + Tree 2 控制，紫色线显示仿真运动轨迹；课程原有的二维差速运动模型保留。物理更新周期 10 ms，双树决策周期 50 ms。地图上的黑线不决定路线，也不代表障碍物。

## 打开与操作

用 Processing 4 打开 `DigitalTwin.pde` 并点击运行。当前电脑也可以在上一级目录的 PowerShell 中运行 `./RunSimulation.ps1`，使用项目 `.tools` 内已从本机安装包解出的 Processing 4.5.7；无需安装 Gazebo。

1. 保持 **Simulation**，点击 **Square demo** 可加载方形示例，或用 **Points / Freehand** 自己画路线。
2. 点击 **Reset pose**，使仿真小车回到原点、车头朝 +X。
3. 点击 **Load into sim → START**。灰色小车执行路线，底部显示 Tree 1 动作、左右 PWM 和路线进度。
4. **STOP / 空格**立即停仿真。修改路线后重新 Load；按 **Reset pose** 后可重跑。

左键添加点、拖动点，右键删除点，Undo 撤销，中键平移，滚轮缩放。每条路线最多 256 个控制点、总长 4 米。车端和仿真均按 1 mm 重采样。STOP 重置进度，属于停止后重启，不是保持进度的暂停。运行中的路线不能编辑；**Load into sim** 可以停止当前仿真并重新加载草稿。

## 实车 TCP

点击 **Physical TCP** 才会尝试连接机器人。默认仿真模式不会连接实车；切换目标前必须停车。实车使用 **Upload route → START**，重新选择目标后需要重新加载或上传。原有 WiFi/TCP 遥测、橙色实车轨迹、G 键遥测图保留。模拟小车不会因收到实车启动信号而自行启动。

## 本次测试结果

已使用真实 Processing 4.5.7 编译器编译完整草图；自动闭环测试直接调用编译后的 `Controller_c`、`RobotSimulation_c`、`MotorsModel_c` 和 `OdometryModel_c`，不使用另一套 Python 动力学。

| 路线 | 严格 5 mm 终点检查 | 仿真用时 | 终点距离 |
|---|---|---:|---:|
| 直线 | 通过 | 5.50 s | 4.750 mm |
| 直角转弯 | 通过 | 7.70 s | 4.901 mm |
| 闭合方形 | 通过 | 15.45 s | 4.903 mm |
| S 形折线 | 通过 | 9.90 s | 4.818 mm |
| 交叉路线 | **未通过**，Tree 1 提前 STOP | 18.25 s | 5.060 mm |
| 初始车头反向、倒车 | 通过 | 7.05 s | 4.875 mm |
| 初始原地转向 | 通过 | 6.60 s | 4.412 mm |
| 含重复点的折线 | 通过 | 6.30 s | 4.752 mm |

本轮 **7/8** 达到严格终点条件。交叉路线并未跳过交叉路段；它在终点附近由已训练 Tree 1 的 STOP 类提前停车，约比 5 mm 条件多 0.06 mm。保留此结果，没有修改阈值或忽略失败。界面会显示 `Tree1 STOP before geometric endpoint`。

Java 与实际 C++ 适配器对照 **11070 组**：动作、进度、目标索引、终点状态、整数 PWM 均无差异；连续几何量最大误差约 1.78e-15。另有 **86 项**鼠标、模式选择和 TCP 协议逻辑检查及原有 3 组 C++ 测试。

机器可读结果和每条路线逐步轨迹见上一级 `simulation-results/report.json` 和同目录 CSV。当前运动模型是课程的理想线性模型，缺少实测摩擦、打滑和时延；这些结果不代表实车成功率。Tree 2 仍使用合成教师数据。

## 重跑检查和同步模型

在上一级项目目录运行（Python 路径按本机布局）：

```powershell
& ../../../second_decision_tree/.venv/Scripts/python.exe tests/run_processing_simulation_checks.py
```

退出码 2 表示测试完成但至少一条路线未达到严格终点条件，失败详情仍会写入报告。其他编译或断言错误会中止检查。

更新 C++ 权重后，先运行 `tools/export_processing_models.py` 生成 `TreeModels.java`，再执行上述完整检查。生成文件记录三个 C++ 源头文件的 SHA256；不要手工更改 Java 权重。`TwoTreeController.java` 实现与固件相同的连续进度、特征和 PWM 适配。
