# Library Patrol Project

AI赋能图书馆微型自主巡检机器人关键代码。

## 本次提交重点

- `library_patrol_final_demo/`

包含最终演示系统代码：
- Web 前端
- 后端任务编排
- 导航客户端
- 语音桥接
- 工单管理
- 摄像头代理
- 图书识别、遗失物检测、高危巡检相关接口代码

## 注意

本仓库不包含 API Key、.env、模型权重、日志文件、编译产物和交叉编译工具链。

如需运行千问视觉 API，请在本地自行创建：

library_patrol_final_demo/.env

并配置：

DASHSCOPE_API_KEY=your_api_key
DASHSCOPE_BASE_URL=your_base_url
QWEN_VL_MODEL=qwen3-vl-flash
