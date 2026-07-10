#!/bin/bash
set -e
cat <<'EOF'
最终演示集成层不会自动启动导航，避免误操作小车。

请按顺序手动执行：

1. 先启动导航工程：
   cd ~/Library_Patrol_Project/robot_2k03011/robot_2k0301
   BOARD_IP=192.168.43.192 ./scripts/start_navigation.sh

2. 再启动本 Web 控制台：
   cd ~/Library_Patrol_Project/library_patrol_final_demo
   ./scripts/start_demo_web.sh

浏览器访问：
   http://127.0.0.1:8090
EOF
