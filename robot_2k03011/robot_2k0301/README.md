# robot_2k0301

这是为“基于龙芯 2K0301 的 AI 赋能图书馆微型自主巡检机器人”准备的新代码骨架。

当前目标是先搭出干净、可扩展的工程结构，再逐步接入 LS2K0301 官方库、底盘硬件、传感器、后台和 ROS2。

## 目录结构

```text
robot_2k0301/
├── board_app/       # 运行在 LS2K0301 板端的 C++ 控制程序
├── backend/         # 本地后台预留目录
├── ros2_ws/         # ROS2 Humble 预留工作空间
├── config/          # 地图、任务、机器人参数
├── docs/            # 设计文档
├── scripts/         # 拉库、构建脚本
└── third_party/     # 第三方库，放 LS2K0301 官方库
```

## 板端模块

```text
board_app/
├── hardware/     # 电机、编码器、摄像头、IMU、ToF、ADC、GPIO 等硬件抽象
├── control/      # PID、底盘控制
├── navigation/   # 拓扑地图、路径规划、节点导航
├── perception/   # 插座巡检、环境采样、低视角失物扫描
├── task/         # 巡检任务状态机
├── comm/         # 与后台或 ROS2 bridge 通信
├── utils/        # 配置、日志等工具
└── main.cpp
```

## 获取 LS2K0301 官方库

官方仓库：

```text
https://gitee.com/seekfree/LS2K0301_Library.git
```

官方库以 Git 子模块放在 `third_party/LS2K0301_Library`。首次下载项目时执行：

```bash
git clone --recurse-submodules https://github.com/NuChen0602/qianrushi.git
```

已经普通克隆项目时，可补充下载子模块：

```bash
git submodule update --init --recursive
```

仓库内 `patches/LS2K0301_Library-local-changes.patch` 保存了本项目对官方库示例和设备树的修改。需要恢复这些修改时执行：

```bash
git -C third_party/LS2K0301_Library apply ../../patches/LS2K0301_Library-local-changes.patch
```

后续更新官方库时执行：

```bash
cd robot_2k0301
./scripts/fetch_ls2k0301_library.sh
```

用户态开源工程路径：

```text
robot_2k0301/third_party/LS2K0301_Library/LS2K030x_Library/Seekfree_LS2K030x_Opensource_Library
```

## 本机骨架构建

不依赖真实硬件，先验证工程结构：

```bash
cd robot_2k0301
./scripts/build_host.sh
```

运行：

```bash
./build/host/board_app/robot_board_app
```

## 2K0301 板端构建

等 LS2K0301 官方库克隆完成、工具链路径确认后：

```bash
cd robot_2k0301
./scripts/build_ls2k0301.sh
```

该脚本会使用官方库中的交叉编译配置：

```text
third_party/LS2K0301_Library/LS2K030x_Library/Seekfree_LS2K030x_Opensource_Library/project/user/cross.cmake
```

当前官方配置里的默认工具链路径是：

```text
/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6
```

如果你的机器上路径不同，需要先修改官方 `cross.cmake` 或新增自己的 toolchain 文件。

## 键盘遥控

```bash
./scripts/start_keyboard_teleop.sh
```

按住 `W/S` 前进或后退，`A/D` 调整舵机，`C` 回正，空格急停，`Q` 安全退出。
超过 650 ms 没有收到行驶按键、电控连接断开或雷达数据超时，程序都会自动停车。

## 已有地图定位

```bash
./scripts/install_localization_deps.sh  # 首次运行一次，无需 sudo
./scripts/start_localization.sh
```

定位使用 `maps/library.yaml` 和 `maps/library.pgm` 固定地图，通过 AMCL 将实时雷达与
墙面持续匹配。启动后需要在 RViz 中用 `2D Pose Estimate` 设置实际初始位姿。

## 固定地图目标点导航

```bash
./scripts/start_navigation.sh
```

先用 `2D Pose Estimate` 对齐小车位姿，再用 `2D Goal Pose` 指定目标。导航节点会在
`maps/library.yaml` 占据栅格上运行 Hybrid A*，按阿克曼自行车模型、车身尺寸和安全
余量生成可行路径；绿色线是 `/planned_path`。小车沿路径前视跟踪，舵机由航向 PID
平滑修正，实时雷达负责近距离停车保护。

导航同时维护一张只供规划器使用的临时障碍地图。雷达在静态地图空闲区域发现的
障碍会保留 2 秒并按车身宽度膨胀；障碍占用当前路径时，小车先停车，再从当前位置
重新运行 Hybrid A* 绕行。AMCL 位姿协方差、雷达和里程计更新时间共同形成定位质量
门控；定位丢失时电机保持停止，质量恢复后重新规划。规划失败、偏离路径或 3 秒没有
运动进展时最多自动恢复 3 次，之后才将任务标记为失败。

路径跟踪器会前视路径曲率，在进入弯道前降低目标速度；速度输出经过加速度、减速度
和加加速度限制，前进/后退切换必须先平滑降到零。导航层按前轮角速度限制转向，TCP
电控桥再按实际舵机角速度做第二层限幅。当前速度、舵机角、前轮角和估算转弯半径可在
Web 上位机查看。控制参数位于 `config/ackermann_navigation.yaml`，硬件转向标定表
位于 `config/odometry.yaml`。

`start_navigation.sh` 每次启动都会自动保存一份运行日志到
`log/navigation/YYYYmmdd_HHMMSS/`。其中 `navigation.launch.log` 是完整终端输出，
`topics.log` 会周期采样 `/planner/status`、`/navigation/status`、`/cmd_vel`、
`/drive/status` 和 `map -> base_link` TF，`ros/` 是 ROS 节点自身日志，
`board_stream_tail.log` 是退出时抓取的板端雷达/里程计日志尾部。需要完整 topic 包时：

```bash
NAV_RECORD_BAG=1 BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
```

生成实测转向标定表：

```bash
python3 scripts/steering_calibration.py \
  --point=-1:80:-28 --point=-0.5:87.5:-14 \
  --point=0:95:0 --point=0.5:110:14 --point=1:125:28
```

每个点的格式为 `归一化转向命令:舵机角度:实际前轮角度`。示例角度只是初始值，
需要用实车测量结果替换。

启动后也可以不用 RViz，直接打开 Web 上位机：

```text
http://127.0.0.1:8080
```

界面中可以查看地图、实时雷达点、车体位姿、规划路径和导航状态。第一次定位时选择
“初始位姿”，在地图上点车的位置并拖动方向；定位贴合后选择“目标点”点击书架间的
目标位置，小车会自动规划阿克曼可行路径并跟踪过去。需要连续巡检时切到“任务点”，
依次点击多个书架检查点，点击“开始巡检”后会逐点导航；遇到异常可用“取消导航”或
“急停”。

## 定距离与定角度动作

```bash
# 编码器控制直行距离，单位为米
./robot_board_app --move forward --distance 0.2 --speed 20 --timeout 15
./robot_board_app --move backward --distance 0.2 --speed 15 --timeout 15

# IMU 控制实际转角；前轮舵机转向，因此小车走弧线而不是原地旋转
./robot_board_app --turn left --angle 30 --speed 15 --timeout 20
./robot_board_app --turn right --angle 30 --speed 15 --timeout 20
```

动作接近目标时自动减速。默认在障碍物距离 800 mm 内减速、500 mm 内停止；
雷达断流、动作超时和终止信号也会触发停车并让舵机回正。

目前 `hardware/robot_hardware.cpp` 中的 LS2K0301 硬件调用还是 TODO，需要根据官方例程确认具体头文件、设备节点和 API 后补齐。

## 第一阶段开发路线

1. 跑通 LS2K0301 官方例程：PWM、GPIO、编码器、ADC、I2C/UART、USB 摄像头。
2. 在 `hardware/` 中替换仿真实现，接入真实硬件。
3. 调通差速底盘速度闭环。
4. 接 ToF/超声波，完成前方障碍停车。
5. 接拓扑地图和视觉地标，完成节点巡航。
6. 接环境传感器、红外测温、低视角摄像头。
7. 做后台任务下发和数据展示。
8. 最后接 ROS2 Humble 或 LLM 任务解析。

## 重要原则

- LLM 只负责把自然语言转成结构化任务，不直接控制底盘。
- 板端负责实时控制、避障、任务状态机和本地缓存。
- 后台负责地图展示、任务下发、异常记录、失物工单和报告。
- 底层硬件接口只写在 `hardware/`，上层算法不要直接调用逐飞库 API。

## 2026-07 导航与雷达调试记录

本节记录当前实车调试后的状态、改动点、已知问题和推荐操作流程。当前板端连接手机热点：

```text
SSID: intro
password: callme11
board_ip: 192.168.123.70
```

### 当前重要结论

- 雷达已经移动到小车中心，ROS 外参按 `base_link == base_laser` 使用：
  - `laser_x: 0.0`
  - `laser_y: 0.0`
  - `laser_yaw: 0.0`
- 当前约定 `0°` 是小车正前方。
- 板端雷达角度已按 LD19/乐动雷达顺时针角转换成 ROS 逆时针角：
  - `raw_angle_to_ros_deg(angle_deg) = (-angle_deg) % 360`
- IMU Z 轴方向相对 ROS yaw 需要反号：
  - `gyro_z_sign: -1.0`
- 导航模式下左右轮目标物理速度保持一致，转向主要靠舵机：
  - 板端 `mapping-drive` 中左右轮都使用同一个 `command_speed_mps`
  - 左右编码器目标计数不同只是 `left/right_counts_per_meter` 标定不同，不是主动差速转弯
- 固定地图文件：
  - `maps/library.yaml`
  - `maps/library.pgm`
  - `maps/library.posegraph`

### 本轮主要改动

1. 雷达与里程计时间戳

   - 板端雷达 TCP JSON 增加 `mono_ns`。
   - 板端里程计 TCP JSON 增加 `mono_ns`。
   - ROS 侧用 `BoardClockMapper` 把板端单调时间映射到 ROS 时间。
   - `/scan` 使用扫描中点时间戳。
   - `/odom` 使用板端时间戳。

2. 雷达角度与去畸变

   - 修正雷达左右镜像问题：LD19 原始顺时针角转换为 ROS 逆时针角。
   - `tcp_laser_scan` 增加可选 deskew，使用当前 `/odom` 速度估计做扫描运动补偿。
   - 目前雷达在中心，所以 deskew 使用：

     ```yaml
     laser_x: 0.0
     laser_y: 0.0
     ```

3. 雷达遮挡配置

   板端当前配置：

   ```yaml
   lidar_front_center_deg: 0
   lidar_self_mask_start_deg: 120
   lidar_self_mask_end_deg: 290
   lidar_self_mask_max_mm: 350
   ```

   注意：如果车体、支架或线束进入雷达扫描平面，需要先物理垫高或重新测遮挡角度；不要随便屏蔽正前方，因为实测时正前方约 40° 可能确实有墙。

4. 建图键盘速度

   键盘控制速度已恢复到较快版本：

   ```yaml
   initial_speed_mps: 0.20
   min_speed_mps: 0.16
   max_speed_mps: 0.28
   speed_step_mps: 0.02
   full_steer_speed_scale: 0.80
   ```

5. 导航路径跟踪

   `goal_navigator` 从单纯航向 PID 增加了横向偏差负反馈：

   ```text
   steering = heading_steering + cross_track_steering
   ```

   关键状态字段：

   ```text
   signed_cross_track_m
   cross_track_steering
   heading_steering
   ```

   当前参数：

   ```yaml
   cross_track_kp: 1.60
   cross_track_steering_limit: 0.25
   goal_yaw_tolerance_deg: 10.0
   relaxed_goal_yaw_tolerance_deg: 45.0
   relaxed_heading_heuristic_weight: 0.12
   start_collision_tolerance: 0.08
   approach_goal_on_failure: true
   approach_goal_tolerance: 0.35
   allow_goal_yaw_fallback: true
   allow_reverse: true
   path_index_search_forward: 8
   planning_timeout_sec: 12.0
   ```

   `2D Goal Pose` 的绿色箭头会优先作为终点车头方向约束。规划器先找满足终点
   朝向的阿克曼路径，必要时允许倒车；如果精确目标因地图空间不足不可达，会扩大到
   目标附近 0.35 m 内继续找可停车位。最后的 fallback 也不会完全忽略角度，终点车身
   朝向必须在 `relaxed_goal_yaw_tolerance_deg` 内；多次角度重规划仍失败时会明确标记为
   `failed: goal_yaw_unreachable`，避免把“位置到了但角度错了”当成完成。
   `planning_timeout_sec` 是严格角度、靠近目标和宽松角度 fallback 共用的总预算；再次点击
   `2D Goal Pose` 会取消旧规划线程，避免旧目标继续占用 CPU。
   如果实车没有碰墙但定位/地图栅格让当前起点轻微压到占用格，规划器会在
   `start_collision_tolerance` 范围内找最近的自由起点继续规划；`/planner/status`
   中的 `start_adjusted` 和 `start_adjustment_distance_m` 可用于确认是否发生了该修正。
   路径跟踪只在当前位置附近向前搜索少量路径点，避免短距离倒车/摆头路径因为自交或贴近终点而直接跳到路径中后段。

6. 导航速度

   当前导航速度比初始保守版本更快：

   ```yaml
   max_speed_mps: 0.26
   min_speed_mps: 0.10
   curvature_speed_gain: 0.28
   min_curve_speed_mps: 0.15
   max_acceleration_mps2: 0.45
   max_deceleration_mps2: 0.45
   max_jerk_mps3: 1.8
   ```

7. 近障停车角度

   以前近障停车默认把 `-90°` 当作前方，这是旧坐标习惯。现在 0° 是正前方，所以导航启动时显式传：

   ```yaml
   front_sector_center_rad: 0.0
   obstacle_stop_distance_m: 0.08
   obstacle_slow_distance_m: 0.35
   ```

   如果日志出现：

   ```text
   remote drive stopped: obstacle 0.07m
   ```

   说明 `tcp_odometry` 近障安全层认为当前运动方向 7 cm 处有障碍，并把速度清零。先检查车头/车尾真实距离，再看 `/drive/status` 的 `stop_reason`。

8. 导航定位方案 A / B

   当前保留两套定位后端，启动接口不变，通过环境变量切换：

   - 方案 A：`slam_toolbox localization`
   - 方案 B：`AMCL`

   默认已经切到方案 A。方案 A 会读取 `maps/library.posegraph`，启动后先用
   雷达扫描与已保存的位姿图做匹配，不需要先点 RViz 的 `2D Pose Estimate`：

   ```bash
   BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
   ```

   回退方案 B：

   ```bash
   NAV_LOCALIZER=amcl BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
   ```

   重要说明：

   - 方案 A 目前只是雏形，接口已接通，但还没有充分实车调好。
   - 方案 A 依赖 `maps/library.posegraph`，不是只用 `.yaml/.pgm`。
   - 方案 B 的 AMCL 在导航运动中发现明显“激光点云飘逸/横向偏移”，但建图时 SLAM scan matching 几乎不飘。这里是短距离几乎不飘，但是转向长距离会飘，你可以运行一下看看效果，然后现在建图是一点没问题，建图质量超高，然后就是问题就是，这个导航，这个舵机偏的不多，估计是那个阿克曼转向有参数出错，需要修正，这个转向你交给AI问一下，我也说不太清楚。目前就是导航不精准，你需要再看一下，说不定方案a是可以，试一下（如果b实在解决不了的话。

### 推荐操作流程

#### 1. 确认板端在线

```bash
ping 192.168.123.70
ssh root@192.168.123.70
```

#### 2. 建图

```bash
cd /home/chen/robot_2k0301
BOARD_IP=192.168.123.70 ./scripts/start_mapping_keyboard.sh
```

操作建议：

- 用键盘慢速绕场地一圈。
- 观察 RViz 中紫色 `/scan` 是否能稳定贴合正在生成的地图。
- 转弯时尽量不要猛打方向；如果雷达拖影明显，先降速再测。

保存地图：

```bash
cd /home/chen/robot_2k0301
./scripts/save_slam_map.sh
```

保存后应至少生成：

```text
maps/library.yaml
maps/library.pgm
maps/library.posegraph
```

当前场地墙内净尺寸固定为 `1.8 m x 1.8 m`。保存脚本会从最大连通空闲区
估算旋转后的墙内净尺寸，默认允许 `+/-0.20 m` 的栅格墙厚和测量误差；占用点
最外包围尺寸只用于检查远离场地的幽灵墙，不再直接与 `1.8 m` 比较。地图被拉伸、存在远离场地的幽灵墙，
或未知区域比例过高时，本次结果会改名为 `*.rejected`，并自动恢复保存前地图。
定位与导航启动脚本也会执行同一检查，坏地图不会直接进入实车导航。
也可以单独检查当前地图：

```bash
python3 scripts/check_map_quality.py --map-yaml maps/library.yaml
```

#### 3. 导航，默认方案 A

```bash
cd /home/chen/robot_2k0301
   BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
```

启动后：

1. 不要先点 `2D Pose Estimate`。
2. 观察紫色 `/scan` 是否在静止时贴合白色地图墙壁。
3. 车体中心就是 `base_link/base_laser`，因为雷达当前安装在小车中心。
4. 如果静止时扫描已经贴合，再用 `2D Goal Pose` 设置目标。
5. 运动中如果紫色扫描逐渐偏离白色墙壁，优先检查里程计/IMU，再考虑重新建图。

如果方案 A 启动失败，优先检查：

```bash
ls -lh maps/library.posegraph
```

#### 4. 回退方案 B：AMCL

```bash
cd /home/chen/robot_2k0301
NAV_LOCALIZER=amcl BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
```

方案 B 的现象：

- 静止时可能看起来能对齐。
- 运动时紫色激光点云可能横向飘、被 AMCL 拉回。
- 建图时不明显，因为建图使用 SLAM scan matching，不是 AMCL 固定地图定位。

如果必须用 B 方案，建议下一步调：

- AMCL 运动模型参数 `alpha1~alpha5`
- `laser_likelihood_max_dist`
- `sigma_hit`
- 是否降低 AMCL 对激光的过度拉扯
- 是否改用更适合 Ackermann 的定位模型或继续走方案 A

#### 5. 观察导航状态

启动导航脚本后会自动保存运行日志，优先查看最新的：

```bash
ls -td log/navigation/* | head -1
```

其中 `topics.log` 重点看 `path_mode`、`approach_goal`、`relaxed_goal_yaw`、
`goal_yaw_error_deg` 和 `requested_goal_yaw_error_deg`。如果第二个 goal 出现
“位置到了但车头差 180°”，把最新整个 `log/navigation/YYYYmmdd_HHMMSS/` 目录用于排查。
当状态为 `aligning_goal_yaw` 时，表示位置已经接近但绿色箭头角度未对齐，系统会重新尝试带终点朝向的规划；
如果最终状态为 `approached` 且原因是 `goal_yaw_unreachable`，表示已接近目标但当前地图/空间下仍无法满足目标朝向。

```bash
cd /home/chen/robot_2k0301
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR=/tmp/ros-log-check
mkdir -p "$ROS_LOG_DIR"

ros2 topic echo --once /navigation/status
ros2 topic echo --once /navigation/localization_status
ros2 topic echo --once /planner/status
ros2 topic echo --once /drive/status
ros2 topic echo --once /cmd_vel
```

重点字段：

```text
/navigation/status
  state
  cross_track_m
  signed_cross_track_m
  curvature_feedforward
  signed_curvature_m_inv
  lookahead_m
  heading_steering
  cross_track_steering
  steering

/drive/status
  requested_speed_mps
  applied_speed_mps
  requested_steering
  applied_steering
  servo_deg
  front_wheel_deg
  turning_radius_m
  stop_reason
```

判断方法：

- `cross_track_steering` 经常顶到 `0.25`：横向偏差修正已经打满，优先怀疑舵角/转弯半径物理能力不足。
- 左弯时 `curvature_feedforward` 应为正；如果 `steering` 接近 `1.0`，`servo_deg` 应逐渐到约 `125`。
- `stop_reason` 是 `obstacle xxm`：近障保护正在清零速度。
- `/cmd_vel` 有速度但 `/drive/status applied_speed_mps` 是 0：多半被近障/scan timeout/cmd timeout 拦住。
- `/planner/status` 报 `start pose collides with the map or map boundary`：初始位姿点到了墙里或车体 footprint 离墙太近。

#### 6. 舵机与转弯半径标定

当前软件假设：

```yaml
servo_right_deg: 80
servo_center_deg: 95
servo_left_deg: 125
steering_wheel_deg_points: [-28.0, -14.0, 0.0, 14.0, 28.0]
wheelbase_m: 0.18
```

这意味着规划器当前按最大前轮角约 28° 规划，最小转弯半径约：

```text
R = 0.18 / tan(28°) ≈ 0.34 m
```

如果实车前轮实际只能打到 20°~25°，真实转弯半径会变成约 0.39~0.49 m，规划出来的弯就会拐不过去。

实测舵机：

```bash
ssh root@192.168.123.70
cd /home/root/robot_2k0301
./robot_board_app --test servo --angle 95
./robot_board_app --test servo --angle 80
./robot_board_app --test servo --angle 125
```

记录：

```text
servo 80  -> 前轮右转实际角度
servo 95  -> 是否正中
servo 125 -> 前轮左转实际角度
```

再用实测值更新：

```yaml
steering_servo_deg_points
steering_wheel_deg_points
max_steer_angle_deg
max_front_wheel_angle_deg
```

#### 7. 常用编译与部署

ROS 包：

```bash
cd /home/chen/robot_2k0301/ros2_ws
colcon build --packages-select robot_lidar_bridge --symlink-install
```

板端程序：

```bash
cd /home/chen/robot_2k0301
./scripts/build_ls2k0301.sh
scp build/ls2k0301/board_app/robot_board_app \
  root@192.168.123.70:/home/root/robot_2k0301/robot_board_app.new
ssh root@192.168.123.70
cd /home/root/robot_2k0301
killall robot_board_app 2>/dev/null || true
cp robot_board_app robot_board_app.previous
mv robot_board_app.new robot_board_app
chmod 700 robot_board_app
```

板端配置：

```bash
scp config/robot.yaml root@192.168.123.70:/home/root/robot_2k0301/config/robot.yaml
```

### 当前已知未完全解决的问题

1. 方案 A 仍是雏形

   `slam_toolbox localization` 已接入导航流程，但还没有充分实车调参。需要重点观察：

   - 是否能稳定发布 `map->odom`
   - `/navigation/localization_status` 是否持续 `ok`
   - 运动中 `/scan` 是否比 AMCL 更贴地图
   - 起步、急转和贴墙时是否出现跳变

2. 方案 B 存在导航时激光飘逸

   AMCL 方案静止时可能能对齐，但运动中紫色点云相对地图有横向偏移、被拉回等现象。建图时不明显，是因为建图模式用 SLAM scan matching 持续优化位姿。

3. 横向偏差负反馈只是路径跟踪补偿，不会修复定位漂移

   `cross_track_steering` 可以让车偏离路径时多打一点舵，但如果 `map->odom` 本身漂移，控制器会追一个被定位误差污染的位置。

4. 转弯物理能力还需实测

   如果路径规划半径小于实车真实最小转弯半径，再强的负反馈也只是打满舵，车仍然会外飘。

## 当前导航使用说明

启动导航：

```bash
BOARD_IP=192.168.123.70 ./scripts/start_navigation.sh
```

启动完成后不需要使用 RViz 的 `2D Pose Estimate`，直接点击 `2D Goal Pose` 设置目标位置和车头方向，小车即可规划路径并导航，终点车头会尽量对齐绿色箭头。

目前已能完成目标点导航，但还有两个问题需要继续优化：

- 系统中的车身尺寸和轮廓还不够准确。
- 规划安全余量已减到 1 cm；目标点精确位姿碰撞时，会在 `goal_tolerance` 内接受可停车位。
注意尽量让小车从我视屏的位置出发，这里设了一个起始点，如果从其他位置出发，就需要进行2d pose estimate进行定位，然后现在建图和导航大概都没有问题，我觉得下一步是增强导航精度，告诉他车辆精确的安全距离（今天有一次因为路径规划没计算好车身距离导致撞墙），然后是倒车需要设计一下，因为如果检测到目标点直接前进无法通过，则会路径规划失败，需要倒车功能），差不多就是这样子，然后就是不要用老对话了，创建一个新的对话，我这个在你这个“配置OPENCV库”这个对话一个命令直接耗费了60的限额，上下文太多了，token消耗过大，






连续第二个目标仍可能规划无解

   最新复现日志：`log/navigation/20260707_075334/`。

   这次测试中，第一个目标已经不是“角度完全没对齐”的老问题。规划器对第一个目标
   `(0.130, -1.240, -177.4deg)` 生成了 `mode=position+yaw`，初始规划结果里的
   `goal_yaw_error=-0.1deg`；控制器经过终点低速修角后，最终以
   `requested_distance=0.089m`、`yaw_error=-8.3deg` 判定到达，说明角度优先参数已经生效。

   失败发生在第一个目标之后。到达后车的位置/朝向约为 `(0.18, -1.15, -172deg)`，
   再点击第二个目标 `(1.025, 0.060, 4.4deg)` 时，规划器连续执行严格角度、靠近目标、
   放宽角度等阶段，每个阶段约 10 秒，仍报：

   ```text
   path planning failed: no exact or approach path found
   ```

   随后换成另一个第二目标 `(1.065, -0.555, 1.2deg)` 也出现同样失败，每个阶段大约
   `1900~2040` 次 expansion 后超时。这个现象说明问题已经不只是
   `planning_timeout_sec` 太短；更可能是当前单次 Hybrid A* 从“贴近书架/墙边且车头朝内”的姿态直接搜索到远处带终点朝向约束的目标时，缺少“先离开狭窄区域、再去目标”的中间机动策略。

   当前判断：

   - 不能简单把终点角度放宽到 90 度或 180 度，因为书架停车时车身方向非常重要。
   - 继续单纯增加规划时间收益有限，只会让第二次点目标等待更久，不保证能找到离场路径。
   - 问题重点已经转为“窄空间离场 + 倒车/多点掉头 + 最终角度约束”的算法问题。

   后续推荐的算法修改方向：

   - 增加分阶段离场规划：直接到第二目标失败时，先在当前车附近搜索一组
     `escape pose`，例如离当前位姿 0.25~0.50 m、碰撞余量更大、能让车离开书架/墙边的中间点；
     先规划到这个中间点，再从中间点重新规划到原始 goal。
   - 增加双向或格点式搜索：从当前位姿和目标位姿集合同时扩展，在中间连接路径；这类方法对
     狭窄停车位、倒车出库、多点掉头通常比当前单向 goal-biased Hybrid A* 更容易找到解。
   - 增强运动原语：当前固定 `primitive_length` 和 5 个转向采样可能漏掉短距离倒车、S 形修正、
     三点掉头等机动；可以在障碍附近使用更短 primitive、更多转向采样，或显式加入倒车出库原语。
   - 保持角度优先但改进候选目标：仍以 `goal_yaw_tolerance_deg: 10.0` 为主目标；如果精确目标不可达，
     在 `approach_goal_tolerance` 内采样多个停车候选点，按“角度误差最小、离原 goal 最近、碰撞余量最大”
     排序，而不是只做一次固定 fallback。
   - 增加失败诊断：规划失败时记录搜索到的最接近目标节点、最大 clearance 节点、失败阶段和 frontier，
     方便判断是地图堵死、转弯半径不足，还是搜索策略没有找到离场动作。


目前第一个目标点已经能较稳定地按 10° 角度容差完成，但连续点击第二个目标时仍可能规划无解。还需要继续优化：
- 从书架/墙边目标离场去第二个目标时，需要增加分阶段离场、倒车出库或多点掉头规划。
