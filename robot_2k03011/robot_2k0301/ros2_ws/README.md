# ROS2 Humble 预留工作空间

当前板端骨架不强依赖 ROS2。建议先跑通 2K0301 板端硬件和本地后台，再把通信桥接到 ROS2。

后续可以添加：

```text
ros2_ws/src/
├── robot_msgs/       # 自定义消息：任务、节点状态、环境数据、告警、工单
├── robot_bridge/     # 板端 TCP/HTTP 数据与 ROS2 topic/service 转换
└── robot_dashboard/  # 可选 ROS2 侧展示或调试节点
```

推荐话题：

- `/robot/state`
- `/robot/sensors`
- `/robot/task`
- `/robot/inspection_event`
- `/robot/lost_item_ticket`

