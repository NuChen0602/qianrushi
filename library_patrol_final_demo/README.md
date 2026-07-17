# library_patrol_final_demo

这是图书馆巡检机器人“最终视频演示集成层”。

它负责统一调度 Web 展示、任务工单、语音命令占位、导航 HTTP API、摄像头 JPEG 代理、图书识别和异步遗失物视觉服务。

本工程不修改导航工程，也不直接发布运动控制命令。导航经现有 HTTP 后端下发；CI302 输入桥读取板端 `/dev/ttyS1`，HS-S77 服务独占 `/dev/ttyS0`。

## 当前功能

- Web 控制台：地图、路径、小车位姿、业务点位、MJPEG 摄像头、实时检测框、工单状态。
- 导航客户端：通过现有 dashboard HTTP API 调用 `/api/goal` 和 `/api/patrol/start`。
- 四类演示任务：
  - 寻书引导
  - 书架错放巡检
  - 全图遗失物巡检
  - 高危点位检查
- 模拟语音命令：A2、A4、A5、A6。
- 组合语音链路：CI302 离线识别“你好小亚”，唤醒后才由 USB 麦克风录音并调用 GLM-ASR。
- ArUco 图书定位：本地 OpenCV 识别书脊标记并在 Web 画框。
- 遗失物视觉：本地 OpenCV 实时候选框和 track ID，HOG+线性 SVM 二分类钥匙校园卡组合/背景。
- 检测框状态：本地模型连续 3 帧确认前显示黄色候选框，确认后显示红色目标框。
- 高危点位检测目前仍为演示占位结果。
- START_HOME：启动后尝试从导航 `/api/state` 的机器人当前位姿自动记录；失败时显示“起点未记录”。
- MQ-2 烟雾检测：板端 A3 ADC 后台采样，连续 3 次达到 1.00V 告警，降至 0.90V 后解除，并在环境状态与工单事件中显示。
- DHT11 温湿度：读取板端内核 IIO 设备，状态接口为 `/api/environment`，断线时保留最近一次成功值并标记 `stale`。

### DHT11 部署

温湿度应用层默认读取 `/sys/bus/iio/devices`，设备名为 `dht11` 或
`dht11_sensor`，温度和相对湿度均按内核 IIO 的 milli 单位转换。压缩包中的
`dht11/source_snapshot/kernel` 和 `patches` 用于内核树合并：启用 `CONFIG_DHT11=y`，
并确认设备树将 DHT11 数据线接到核心板 P89（gpa5 9）。未部署内核驱动时，Web 会显示传感器离线，
不会影响导航和其他演示功能。

## 视觉低延迟链路

```text
板端 JPEG 流 -> CameraProxy 单连接缓存 -> Web MJPEG（上限 12 FPS）
                                      -> OpenCV 本地候选/track_id（10 FPS）
                                      -> 本地 HOG+SVM 二分类（96×96）
                                      -> 连续 3 帧确认 -> 红框
                                      -> 模型缺失时才使用单槽 Qwen 队列兜底
```

- 浏览器不再每 500 ms 重建一次 JPEG 请求，而是保持一个 `/camera.mjpg` 连接。
- 本地模型只依赖 OpenCV；连续 3 帧判为目标才确认，连续 3 帧判为背景则隐藏候选。
- 当前部署模型按现场实时画面将判定阈值校准为 `-0.30`；路线完成后仍保持遗失物扫描和检测框显示，直到切换到其他任务。
- 横向占比明显增大的近距离目标使用尺寸自适应阈值，最低限制为 `-0.95`；连续 4 个新帧未匹配就删除旧框，每帧最多显示一个钥匙串检测框。
- 本地模型文件缺失时，Qwen请求才在独立线程运行；等待队列长度固定为1。
- 检测坐标只保留约 1.5 秒；物品类别绑定 `track_id`，框坐标始终由本地检测持续更新。
- 默认 `QWEN_VL_MAX_TOKENS=80`，只发送候选区域，不发送整张 640 像素画面。

## 启动方式

完整演示推荐使用总启动脚本：

```bash
cd ~/Library_Patrol_Project/library_patrol_final_demo
./scripts/start_full_demo.sh
```

该脚本会依次等待导航就绪，关闭会占用 `/dev/video0` 的 IPS200 摄像头显示，
通过 SSH 启动板端 `board_stream_server`，然后启动 Web、视觉服务和语音桥。
保持这个终端开启，按一次 `Ctrl+C` 会统一停止导航、板端视频流、Web、
遗失物视觉服务和语音桥。默认参数为：

- 板端：`192.168.43.192`
- 板端摄像头：`/dev/video0`，`640×480`、15 FPS、5000 端口
- 语音串口：`/dev/ttyS1`，`115200` 波特率
- 导航 API：`http://127.0.0.1:8080`
- 演示 Web：`http://127.0.0.1:8090`

如需修改参数，可在启动命令前设置环境变量，例如：

```bash
BOARD_IP=192.168.43.192 VOICE_SERIAL=/dev/ttyS1 ./scripts/start_full_demo.sh
```

如板端摄像头设备号发生变化，可使用 `CAMERA_DEVICE=/dev/video1` 覆盖。

其中 Web 启动脚本会同时启动：

- `8090`：演示 Web、导航代理和 MJPEG。
- `8091`：异步遗失物视觉状态服务。

首次运行需要准备 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

启用自然语言录音服务（需安装 `arecord`，并先启动演示 Web）。该服务不持续录音，等待 CI302 把“你好小亚”的唤醒帧转发到本机 `127.0.0.1:8092/wake` 后，才录制下一句话并调用 GLM-ASR：

```bash
python3 scripts/voice_q_record_transcribe.py --device default
```

总启动脚本会自动启动录音服务和 CI302 输入桥。当前小车 USB 麦克风使用稳定设备名 `plughw:CARD=Device,DEV=0`，启动时自动设为 100% Capture 并开启自动增益；可通过 `VOICE_DEVICE`、`VOICE_MIXER_CARD`、`VOICE_MIC_GAIN` 和 `VOICE_AGC` 覆盖。当前 Jieli USB 麦克风关闭自动增益时可能输出全零音频。CI302 唤醒帧为 `AA 55 03 00 FB`；总启动使用 `--input-only`，CI302 只识别和发码。

HS-S77 使用后端启动时建立的持久 SSH 双向 UART 连接。每条播报严格等待模块 `0x41` 接收状态和 `0x4F` 播放完成状态；`0x45`、超时和断线会明确失败，断线会自动重连。

### CI302 标准命令优先与 API 兜底

新烧录表为 `/home/chen/图书馆巡检_CI302标准命令_API兜底_HSS77播报词库.xlsx`，由原最大词库模板生成。新表不包含任何 CI302 播报语，所有“播报语句”单元格均为空。生成命令：

```bash
python3 scripts/generate_ci302_command_workbook.py \
  /home/chen/图书馆巡检_四部分视频导航版_再瘦身可烧录词库.xlsx \
  /home/chen/图书馆巡检_CI302标准命令_API兜底_HSS77播报词库.xlsx
```

唤醒后至 CI302 休眠前，USB 麦克风按短片段监控：

1. CI302 命中任一标准词（包括 15 个“寻找图书”词）时发码，统一调度器使旧任务失效，并在确认音结束后直接执行预写任务，不调用 ASR/API。
2. 只有 CI302 没有发出标准命令码、本地 VAD 又确认录到了人声时，才调用 GLM-ASR 和对话 API。
3. 静音片段不会上传。CI302 发出 `AA 55 02 6F FB`（同时兼容 `AA 55 02 00 FB`）后，只结束麦克风/API会话；HS-S77 不播报休眠语，也不取消已开始的机器人任务。

A1–A9 保持原视频协议；其余 14 本图书使用 B0–BD，“退出对话”使用 BE。完整映射由 `app/ci302_commands.py` 同时提供给生成器和运行代码，避免表格与程序不一致。

当前烧录表会让 CI302 在标准命令命中后先播报“好的”。固件没有播放完成帧，因此唯一的降级参数 `CI302_ACK_FALLBACK_SECONDS` 默认保守等待 `1.0` 秒；HS-S77 本身不使用固定延时。唤醒提示固定为“请说”，且只有其真实播放完成后才开始录音。

正式模式默认不保存录音或转写；仅在显式设置 `VOICE_SAVE_RECORDINGS=1` 或 `VOICE_DEBUG_TRANSCRIPTS=1` 时保存，并使用仅当前用户可读权限及数量/时间清理策略。
API Key 只从 `ZHIPU_API_KEY` 读取，缺失时总启动脚本会拒绝启动；源码和命令行中不再携带密钥。

### 可查找图书与 ArUco 码

完整关系由 `LS2K0301_Library/aruco_board_stream/book_database.json` 提供，共 15 本：

| 书架 | ArUco ID | 书名 |
|---|---:|---|
| A1 工科 | 101 | 工程控制论（上册）（第三版） |
| A1 工科 | 102 | 机器视觉 |
| A1 工科 | 103 | 微型计算机系统原理及应用（第3版） |
| A1 工科 | 104 | 现代工程技术与创新实践 |
| A1 工科 | 105 | 电路（第6版） |
| A2 理科 | 151 | 工科数学分析（上册） |
| A2 理科 | 152 | 科研人的自我修养 |
| A2 理科 | 153 | 物理学（第七版）上册 |
| A2 理科 | 154 | 物理学（第七版）下册 |
| A2 理科 | 155 | 量子简史 |
| A3 文学历史 | 201 | 苏东坡传 |
| A3 文学历史 | 202 | 霍乱时期的爱情 |
| A3 文学历史 | 203 | 百年孤独 |
| A3 文学历史 | 204 | 瓦尔登湖 |
| A3 文学历史 | 205 | 大国崛起 |

对话示例包括“帮我找瓦尔登那个湖”“带我去找一本讲量子发展史的书”“介绍一下机器视觉”。API 只能从上述目录返回有效 ID，无法确定时会继续追问，不会生成不存在的导航目标。

语音指令按三级优先级处理：

1. 标准指令精确匹配：如“寻找百年孤独”“介绍这本书”“检查当前书架”，直接执行预写程序，不调用对话 API。
2. 本地确定性匹配：如“找一下百年孤独”“介绍一下机器视觉”，直接由代码识别，寻找安全门要求明确寻找词、无否定且书名唯一。
3. API 模糊兜底：前两级都不匹配时，API 只返回意图、标准命令 ID、图书 ID或短回复；图书介绍始终取本地固定知识库。

原视频中的推荐图书、书架巡检、遗失物巡检、高危巡检、返回起点和停止任务仍保留固定任务代码；所有播报文本最终统一交给 HS-S77。完成寻书后说“介绍这本书”，会介绍最近一次寻找到的目标图书。

如需启用 Qwen-VL，在 `library_patrol_final_demo/.env` 中设置：

```bash
DASHSCOPE_API_KEY=your_api_key
DASHSCOPE_BASE_URL=your_openai_compatible_base_url
QWEN_VL_MODEL=qwen3-vl-flash
```

浏览器访问：

```text
http://127.0.0.1:8090
```

也可以先只启动本 Web。此时页面会显示导航未连接，按钮可触发任务框架，但真实导航下发会失败并写入工单错误。

## 调试方式

### 录制本地分类数据

板端 `board_stream_server` 当前只允许一个客户端连接。直接录制时先停止 Web，避免
`CameraProxy` 占用板端流。

录制钥匙与校园卡组合正样本：

```bash
./scripts/record_dataset_video.py \
  --label target_bundle \
  --duration 60 \
  --video-fps 15 \
  --sample-fps 3
```

移走目标、摆入纸片/普通卡片/电线等干扰物后录制负样本：

```bash
./scripts/record_dataset_video.py \
  --label background \
  --duration 60 \
  --video-fps 15 \
  --sample-fps 3
```

输出保存在 `dataset_recordings/<label>/<时间>/`：`capture.mp4` 是完整录像，
`frames/` 是按每秒 3 张自动抽取的训练帧。该目录已加入 `.gitignore`。

自动生成候选裁剪数据集，无需 LabelImg：

```bash
./scripts/build_candidate_dataset.py
```

脚本按完整录像段划分训练集和验证集，正样本通过候选框时间连续性选择，背景帧中的
OpenCV候选全部作为难负样本。随后执行净化和本地模型训练：

```bash
./scripts/train_local_bundle_classifier.py
```

训练器先用可信裁剪自动剔除错误正样本，在独立录像段上评估，再使用全部净化数据重训
部署模型。输出为：

```text
local_models/lost_item_hog_svm.xml
local_models/lost_item_hog_svm.json
```

实时视觉服务启动时会自动加载该模型。录像、候选数据集和模型目录均已加入
`.gitignore`，避免把现场数据或生成模型提交到仓库。

模拟“语音 A2 -> 寻找百年孤独”：

1. 打开 `http://127.0.0.1:8090`。
2. 点击“调试模拟语音”里的“模拟 A2”。
3. 右侧工单应显示“寻书引导”，语音只播报“收到。”；避免将书名或“寻找”再次播出而触发 CI302。
4. 如果导航已启动，确认播报完成后会下发 `LIT_SHELF_A3` 目标点；到达后连续检测目标 ArUco。无帧、异常或超时只会报告失败，生产模式不会回退到成功 Stub。

测试“按钮 -> 书架错放巡检”：

1. 点击“书架错放巡检”。
2. Web 会调用本服务 `/api/demo/mission`。
3. 编排器会向导航 dashboard 调用 `/api/patrol/start`，路线为工科、理科、中间点、文学书架。
4. 到达工科/文学书架时，会写入对应错放工单并触发播报占位。

## 当前占位部分

- `app/voice_ci1302.py`：只支持模拟命令，不打开 `/dev/ttyS1`。
- `vision/aruco_book_detector.py`：已实现真实 ArUco；摄像头不可用时任务编排器仍会回退到固定结果。
- Web 遗失物检测已接入本地 HOG+SVM；`MissionOrchestrator` 的遗失物工单内容仍使用 `vision/lost_item_detector.py` 的固定结果。
- `vision/hazard_detector.py`：按 mode 固定返回高危结果。
- `vision/visual_api_client.py`：旧占位客户端；当前遗失物 Qwen 实现在 `scripts/lost_item_visual_api.py`。

## 后续接入位置

- CI302 唤醒由 `scripts/voice_trigger_ssh_bridge.py` 读取板端 `/dev/ttyS1` 并转发；`app/voice_ci1302.py` 仍只用于 Web 模拟按钮。
- 遗失物闭环工单：让 `MissionOrchestrator` 消费按 `track_id` 确认后的本地模型结果，替换固定工单。
- Qwen-VL 高危检测：复用异步裁剪与本地跟踪架构，实现插座、线缆和出口阻塞识别。
- 新增点位和路线：修改 `config/points.json`、`config/missions.json`。

## 安全边界

- 本工程不会执行 `colcon build`、`cmake`、`make` 或旧工程脚本。
- 本工程不会启动导航，只调用已存在的 dashboard HTTP API。
- 本工程不直接发布 `/cmd_vel`；CI302 与 HS-S77 串口均由各自单一进程持有，总启动脚本会按需启动并回收板端摄像头服务。
