# 本地后台预留目录

后台建议运行在本地笔记本，用于完成：

- 拓扑地图显示
- 巡检任务下发
- 小车状态显示
- 用电异常记录
- 环境数据可视化
- 失物招领工单管理
- 文本/语音任务输入
- 规则解析或轻量 LLM 任务解析

第一阶段可以先用 Python FastAPI + 简单网页实现：

```text
backend/
├── api/          # HTTP/WebSocket API
├── web/          # 前端页面
├── llm_parser/   # 自然语言到结构化任务
└── data/         # 日志、图片、工单缓存
```

建议通信协议先保持简单：

```json
{
  "task": "socket_inspection",
  "target_node": "SOCKET"
}
```

