# RoboticsScienceAndSystems
Platforms for teaching Robotics Science and Systems

## 两棵决策树与鼠标路线控制

用 Processing 打开 `DigitalTwin/DigitalTwin.pde`，连接机器人 WiFi，在画布中点击或自由画线，依次点 **Upload route → START**。**STOP / 空格**停车。路线由 `StampC3_Template` 中的 Tree1 + Tree2 C++ 执行。

完整启动步骤、鼠标操作、坐标、模型限制和编译方法见 [中文使用说明](StampC3_Template/README_AI.md)。

## Processing 4 仿真测试

`DigitalTwin` 现在默认使用双树仿真。打开后依次点击 **Square demo → Reset pose → Load into sim → START**，或自己用鼠标绘制路线。**Physical TCP** 切换到实车上传和遥测模式。也可在本目录运行 `./RunSimulation.ps1`。

测试结果为 8 条路线中 7 条达到严格 5 mm 终点条件，交叉路线在 5.06 mm 处被 Tree 1 提前停车。操作、限制和重跑方法见 [仿真说明](DigitalTwin/README_SIMULATION.md)，详细结果见 [测试报告](simulation-results/report.json)。
