# ArUco Board Stream 联调工具

本目录用于“龙芯板端摄像头采集 + Wi-Fi 发送 JPEG + 电脑端 ArUco
检测”的独立联调。它不依赖 `E08_04` 的 `user/main.cpp`：

- `board_stream_server` 在龙芯 2K0301 板端运行，只采集摄像头、压缩 JPEG
  并通过 TCP 发送。
- `pc_aruco_stream_client.py` 在 Ubuntu 电脑运行，接收画面、检测 ArUco、
  从左到右编号并判断错放。

传输协议为：4 字节网络字节序无符号 JPEG 长度，随后紧跟 JPEG 数据。

## 板端程序交叉编译

以下命令在 Ubuntu 电脑终端执行。先退出 conda，再使用现有龙芯工具链和
板端 OpenCV：

```bash
cd /home/fangzhou/Library_Patrol_Project/LS2K0301_Library/LS2K030x_Library/Example/Motherboard_Demo/E8_camera/E08_04_shelf_book_vision_demo/host_tools/aruco_board_stream

source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh
conda deactivate 2>/dev/null || true

mkdir -p build_board
cd build_board
cmake .. \
  -DCMAKE_CXX_COMPILER=/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/bin/loongarch64-linux-gnu-g++ \
  -DOpenCV_DIR=/opt/ls_2k0300_env/opencv_4_10_build/lib/cmake/opencv4
make -j$(nproc)
```

这套构建不使用、也不执行 `user/build.sh`。

## 上传到板子

电脑和板子接入同一 Wi-Fi 后，将下面 IP 替换成板子的实际地址：

```bash
scp -O board_stream_server root@192.168.43.192:/home/root/
```

## 板端运行

通过串口或 SSH 进入板端后运行：

```sh
chmod +x /home/root/board_stream_server
/home/root/board_stream_server \
  --camera /dev/video0 \
  --width 320 \
  --height 240 \
  --fps 15 \
  --port 5000 \
  --jpeg-quality 80 \
  --rotate 0
```

程序监听 `0.0.0.0:5000`。电脑客户端断开后，服务端会继续等待下一次连接。
若摄像头安装倒置，可把 `--rotate` 改为 `180`。

## 电脑端运行

电脑端需要 Python 3、NumPy 和带 ArUco 模块的 OpenCV。如果
`cv2.aruco` 不存在，安装：

```bash
pip install opencv-contrib-python
```

启动客户端：

```bash
cd /home/fangzhou/Library_Patrol_Project/LS2K0301_Library/LS2K030x_Library/Example/Motherboard_Demo/E8_camera/E08_04_shelf_book_vision_demo/host_tools/aruco_board_stream

python3 pc_aruco_stream_client.py \
  --host 192.168.43.192 \
  --port 5000 \
  --dict 5x5_250 \
  --expected engineering \
  --save-dir outputs
```

ArUco ID 分类规则：

- `101~105`：`engineering`，工科技术类
- `151~155`：`science`，理学科普类
- `201~205`：`liberal`，文学历史类
- 其他 ID：`unknown`

按键：

- `1`：期望类别设为 engineering
- `2`：期望类别设为 science
- `3`：期望类别设为 liberal
- `0`：期望类别设为 unknown
- `s`：保存当前带标注画面
- `q` 或 `Esc`：退出

当期望类别不是 `unknown` 时，类别不同的标记会显示为 `WRONG`，错放编号
按画面中标记中心点从左到右计算。
