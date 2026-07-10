# 软件架构草案

## 数据流

```text
后台任务 / 按钮任务
        ↓
TaskManager
        ↓
PathPlanner + TopologyMap
        ↓
RobotHardware
        ↓
LS2K0301 官方库 / Linux 设备节点
```

巡检数据反向上传：

```text
RobotHardware / Inspection
        ↓
BackendClient
        ↓
本地后台 / ROS2 bridge
```

## 任务类型

- `FullInspection`：全区域巡检
- `SocketInspection`：插座巡检
- `EnvironmentSample`：环境采样
- `LostItemScan`：失物扫描
- `Guide`：寻书引导
- `ReturnHome`：返回起点

## 硬件适配边界

所有 LS2K0301 官方库调用集中放在：

```text
board_app/hardware/
```

后续只需要替换：

- `RobotHardware::initialize()`
- `RobotHardware::update()`
- `RobotHardware::applyMotorControl()`
- `RobotHardware::captureImage()`
- `RobotHardware::beep()`
- `RobotHardware::setStatusLed()`

上层任务、导航、巡检逻辑不直接依赖底层设备 API。

