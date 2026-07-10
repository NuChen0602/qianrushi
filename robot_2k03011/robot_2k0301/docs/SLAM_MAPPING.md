# 编码器里程计与雷达 SLAM

## 1. 安装电脑端组件

```bash
sudo apt install ros-humble-slam-toolbox
```

## 2. 标定编码器距离

板端启动数据流：

```bash
ssh root@192.168.154.70 \
  'cd /home/root/robot_2k0301 && ./robot_board_app --test odom-stream'
```

电脑另开终端。将车轮落地，沿直线推动小车准确走 1 米，结束时按 `Ctrl+C`：

```bash
cd /home/chen/robot_2k0301
./scripts/calibrate_encoder_odometry.py --distance 1.0
```

把输出的 `counts_per_meter` 写入：

```text
ros2_ws/src/robot_lidar_bridge/config/odometry.yaml
```

实际使用值以 `config/odometry.yaml` 中的左右轮参数为准。更换轮胎、编码器、
传动结构或明显调整胎压后需要重新标定。

左右计数应同号且数值接近。如果平均值接近 0，说明其中一路编码器方向符号错误，不能继续建图。

## 3. 建图

```bash
cd /home/chen/robot_2k0301
./scripts/start_mapping.sh
```

启动脚本会先检查：

- 雷达 TCP 数据连续、扫描频率正常且有效点数足够；
- 里程计 TCP 数据连续，IMU 已就绪，没有编码器突跳；
- 预检时小车保持静止，编码器不能持续产生大计数；
- ROS 侧 `/scan`、`/odom`、`odom -> base_link` 和
  `base_link -> base_laser` 全部正常。

任一检查失败时不会启动 SLAM，先根据终端错误处理传感器或网络问题。
建图开始后看门节点会继续监控这些输入；若连续失效，会终止本次建图，防止网络
恢复后用缺失的里程数据继续生成错误地图。

初次建图先用手缓慢推动小车，转弯要慢，并尽量回到已经走过的位置形成闭环。

> 建图期间不要同时运行 `--route`、`--move`、`--turn`、键盘遥控或另一个
> 编码器测试程序。当前 `odom-stream` 会周期性读取并清零编码器，多个板端进程
> 同时读取会导致里程计丢计数，地图会拉伸或重叠。

也可以使用不会争抢编码器的联合键盘建图模式：

```bash
./scripts/start_mapping_keyboard.sh
```

按键为 `W/S` 或上下方向键前后、`A/D` 或左右方向键转向、`C` 回正、
`+/-` 调速、空格停车。默认速度为 `0.26 m/s`，可在 `0.16-0.38 m/s`
范围内调整。
轻按一次 `W/S` 后会保持运动约 2 秒，期间按 `A/D/C/+/-` 会续期；连续
2 秒没有控制按键会自动停车。按 `Q` 后程序会停车、
保存地图并关闭板端数据流。该模式使用同一个板端进程完成电机闭环和里程计采集，
不要再同时运行旧的 `start_keyboard_teleop.sh` 或其他运动测试。
键盘建图模式默认不使用雷达急停，只保留空格/Q停车和命令超时停车，避免车身或
近处点云把前进命令过早拦掉。建图时请把手放在空格键附近。

板端地址变化时可临时指定：

```bash
BOARD_IP=192.168.154.70 ./scripts/start_mapping.sh
```

保存 SLAM 位姿图：

```bash
./scripts/save_slam_map.sh
```

默认保存 `maps/library.posegraph`，并同时生成 `maps/library.yaml` 和
`maps/library.pgm`。覆盖已有地图前脚本会自动创建带时间戳的备份，并检查三个
输出文件均已成功生成。

## 4. 定位

```bash
./scripts/install_localization_deps.sh  # 首次运行一次，无需 sudo
./scripts/start_localization.sh
```

定位使用固定的 `maps/library.yaml`/`library.pgm` 地图和 AMCL 粒子滤波，地图在定位
过程中不会被改写。在 RViz 中使用 `2D Pose Estimate` 指定小车在地图中的初始位置
和朝向，然后低速移动约 20 cm，确认实时雷达点持续贴合墙面。

## 5. 已有地图目标点导航

```bash
./scripts/start_navigation.sh
```

启动后会加载固定地图，打开 RViz、Web 上位机，并启动目标点导航节点。

RViz 操作顺序：

1. 在 RViz 中用 `2D Pose Estimate` 标出小车当前位姿，箭头方向要和车头一致。
2. 确认红色雷达点云能贴合地图墙面。
3. 使用 `2D Goal Pose` 点一个近距离目标点，先从 `0.5 m` 内的小目标开始。
4. 小车到达目标点附近会自动停车；需要急停时关闭终端或在另一个终端执行
   `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`。

目标点导航使用 Hybrid A* 在固定占据栅格上规划路径。搜索状态包含 `x/y/yaw`，
并通过自行车模型生成阿克曼转向运动原语；碰撞检测按 `0.26 m x 0.135 m`
车身和安全余量执行。规划结果发布到 `/planned_path`，RViz 中显示为绿色路径，
随后由前视路径跟踪和 IMU 航向 PID 控制舵机。运行时仍保留实时雷达近距离停车保护。

导航期间 `/navigation/planning_map` 是静态地图与实时临时障碍的合成结果，只供
Hybrid A* 使用；AMCL 始终订阅原始 `/map`，避免移动人员或推车污染定位。安全监督
节点会发布：

- `/navigation/localization_status`：定位质量、协方差和传感器数据新鲜度；
- `/navigation/obstacle_status`：动态点数量及当前路径是否受阻；
- `/navigation/localization_ok`：路径跟踪许可；
- `/navigation/path_blocked`：停车和绕行重规划触发信号。

遇到动态障碍时小车先停车，障碍持续存在超过 0.8 秒后重新规划；规划失败、路径
偏离或无运动进展最多重试 3 次。定位质量丢失时不会继续发送运动命令，恢复后也会
先重新规划再行驶。参数集中在 `config/ackermann_navigation.yaml`。

### 平滑运动与转向标定

跟踪器会查看车前约 `0.45 m` 的路径曲率，在转弯前主动降低速度。最终速度还会经过
最大加速度、最大减速度和加加速度限制，接近目标时按剩余制动距离继续降速。相关参数：

- `curvature_lookahead_m`、`curvature_speed_gain`：弯道预判距离和减速强度；
- `max_acceleration_mps2`、`max_deceleration_mps2`：加减速上限；
- `max_jerk_mps3`：加速度变化速度，越小越柔和；
- `max_steering_rate_deg_per_sec`：导航层实际前轮角速度上限；
- `max_servo_rate_deg_per_sec`：电控桥实际舵机角速度上限。

`config/odometry.yaml` 中的转向表必须按实车标定。停车并架起车身，依次执行：

```bash
./robot_board_app --test servo --angle 80
./robot_board_app --test servo --angle 87.5
./robot_board_app --test servo --angle 95
./robot_board_app --test servo --angle 107.5
./robot_board_app --test servo --angle 120
```

记录每个舵机角对应的实际前轮角，右转记负、左转记正。然后在电脑执行：

```bash
python3 scripts/steering_calibration.py \
  --point=-1:80:-37 --point=-0.5:87.5:-18.5 \
  --point=0:95:0 --point=0.5:107.5:18.5 --point=1:120:37
```

将工具输出写回 `config/odometry.yaml`。也可以低速画圆测量转弯半径，再用
`前轮角 = atan(轴距 / 转弯半径)` 计算角度。Web 上位机和 `/drive/status` 会显示当前
舵机角、前轮角及估算半径，便于确认标定是否一致。

如果目标点落在墙壁、未知区域，或当前位姿与地图发生碰撞，规划器会拒绝启动并保持停车。
首次测试建议在车头前方 `0.4-0.8 m` 的空地点击目标，再逐步测试绕过固定障碍。

Web 上位机访问地址：

```text
http://127.0.0.1:8080
```

在 Web 界面中可以直接看地图、实时雷达点、规划路径、小车位姿、规划器状态和导航状态。
选择“初始位姿”后在地图上点车体位置并拖动方向；选择“目标点”后点击目标书架或通道位置；
选择“巡检点”后可连续添加多个任务点并点击“开始巡检”。急停、解除急停、取消导航和
停止巡检都可以在右侧控制面板完成。

## 建图前检查清单

1. 雷达固定牢靠，车身支架遮挡角度已经在板端过滤。
2. 小车静止时 `/odom` 位置不持续漂移，IMU 零偏标定期间不要碰车。
3. 直推 0.8 m 时 RViz 中里程计距离接近 0.8 m。
4. 原地改变车头方向时，RViz 中 `base_link` 朝向与实车同向变化。
5. 雷达看到的墙面在小车静止时不抖动、不旋转。
6. 建图时低速移动，避免急加速、车轮打滑和抬起车身。

## 坐标约定

- `map -> odom`：定位时由 AMCL 发布，建图时由 SLAM Toolbox 发布。
- `odom -> base_link`：编码器距离与 IMU660RA 航向融合后发布。
- `base_link -> base_laser`：雷达位于车体前方约 0.12 m。
- 雷达原始 0 度方向对应车头。LD19 原始角度按顺时针增长，ROS
  `LaserScan` 桥接时将其转换为逆时针角度；`base_laser` 与 `base_link`
  朝向一致。
