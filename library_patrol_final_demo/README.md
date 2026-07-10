# library_patrol_final_demo

这是图书馆巡检机器人“最终视频演示集成层”。

它负责统一调度 Web 展示、任务工单、语音命令占位、导航 HTTP API、摄像头 JPEG 代理、图书识别和异步遗失物视觉服务。

本工程不修改导航工程，不启动雷达和串口，也不直接发布运动控制命令。摄像头代理会连接已经在板端运行的 TCP/JPEG 服务。

## 当前功能

- Web 控制台：地图、路径、小车位姿、业务点位、MJPEG 摄像头、实时检测框、工单状态。
- 导航客户端：通过现有 dashboard HTTP API 调用 `/api/goal` 和 `/api/patrol/start`。
- 四类演示任务：
  - 寻书引导
  - 书架错放巡检
  - 全图遗失物巡检
  - 高危点位检查
- 模拟语音命令：A2、A4、A5、A6。
- ArUco 图书定位：本地 OpenCV 识别书脊标记并在 Web 画框。
- 遗失物视觉：本地 OpenCV 实时候选框和 track ID，HOG+线性 SVM 二分类钥匙校园卡组合/背景。
- 检测框状态：本地模型连续 3 帧确认前显示黄色候选框，确认后显示红色目标框。
- 高危点位检测目前仍为演示占位结果。
- START_HOME：启动后尝试从导航 `/api/state` 的机器人当前位姿自动记录；失败时显示“起点未记录”。

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
3. 右侧工单应显示“寻书引导”，并记录“正在查找百年孤独”的播报占位。
4. 如果导航已启动，会向导航 dashboard 下发 `LIT_SHELF_A3` 目标点；到达后使用当前摄像头帧执行 ArUco 图书识别，无帧或识别异常时才回退到演示结果。

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

- 真实语音 CI1302：实现 `app/voice_ci1302.py` 的 `VoiceCi1302Serial`，并在 `paths.json` 中开启 `voice.enabled`。
- 遗失物闭环工单：让 `MissionOrchestrator` 消费按 `track_id` 确认后的本地模型结果，替换固定工单。
- Qwen-VL 高危检测：复用异步裁剪与本地跟踪架构，实现插座、线缆和出口阻塞识别。
- 新增点位和路线：修改 `config/points.json`、`config/missions.json`。

## 安全边界

- 本工程不会执行 `colcon build`、`cmake`、`make` 或旧工程脚本。
- 本工程不会启动导航，只调用已存在的 dashboard HTTP API。
- 本工程不会写串口、不会启动板端摄像头服务、不会直接发布 `/cmd_vel`。
