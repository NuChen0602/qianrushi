#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from openai import OpenAI
import httpx
import json
import os
from patrol_knowledge import load_library_map, lookup_semantic_target

class LLMNavigatorNode(Node):
    def __init__(self):
        super().__init__('llm_navigator_node')
        
        # 保留 /goal_pose 发布，便于调试；正式导航使用 Nav2 NavigateToPose action。
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 初始化 DeepSeek 客户端 (兼容 OpenAI 接口)
        self.api_key = os.getenv("DEEPSEEK_API_KEY")
        # 强制创建一个不使用任何系统代理的干净网络客户端
        # trust_env=False 的意思是：不要去读系统里的 http_proxy/all_proxy 环境变量！
        custom_http_client = httpx.Client(trust_env=False)
        
        self.client = None
        if self.api_key:
            self.client = OpenAI(
                api_key=self.api_key,
                base_url="https://api.deepseek.com",
                http_client=custom_http_client
            )
        else:
            self.get_logger().warn("未设置 DEEPSEEK_API_KEY，将使用本地 Mock LLM。")
        self.library_map = load_library_map()
        self.library_map_prompt = json.dumps(self.library_map, ensure_ascii=False)
        self.get_logger().info('🤖 LLM 智能交互与导航节点已启动！')

    def ask_llm_for_coordinates(self, user_input):
        self.get_logger().info(f'正在思考用户的需求: "{user_input}" ...')
        
        # Prompt Engineering (提示词工程)
        system_prompt = f"""你是一个图书馆的智能巡检机器人。
        你的任务是根据用户的自然语言请求，结合[馆藏与语义地图]，推理出目标坐标。
        {self.library_map_prompt}
        请直接输出JSON格式，不要包含任何其他文字。格式严格为：{{"x": 2.5, "y": 1.0}}"""

        try:
            if self.client is None:
                raise RuntimeError("DEEPSEEK_API_KEY is not set")

            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_input}
                ],
                response_format={"type": "json_object"} # 强制要求返回 JSON
            )
            
            # 解析 LLM 返回的 JSON
            result_json = json.loads(response.choices[0].message.content)
            return result_json['x'], result_json['y']
            
        except Exception as e:
            self.get_logger().error(f"调用大模型失败: {e}")
            self.get_logger().warn("已切换至本地备用大脑 (Mock LLM)。")
            
            target = lookup_semantic_target(user_input, self.library_map)
            return target.x, target.y

    def send_navigation_goal(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置目标坐标
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        # 简单设置朝向 (四元数)
        goal_msg.pose.orientation.w = 1.0 
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'目标坐标 (X: {x}, Y: {y}) 已发布到 /goal_pose，准备调用 Nav2 action。')

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 navigate_to_pose action server 未就绪。请检查 Nav2 lifecycle 和 TF。")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        send_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done():
            self.get_logger().error("发送导航目标超时。")
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了目标点。通常是 TF、costmap 或目标点不可达。")
            return

        self.get_logger().info("Nav2 已接受目标点，小车应开始规划并输出 /cmd_vel。")

def main(args=None):
    rclpy.init(args=args)
    navigator = LLMNavigatorNode()
    
    try:
        while rclpy.ok():
            # 开启一个终端交互循环
            user_text = input("\n🎤 请输入您的自然语言指令 (输入 q 退出): ")
            if user_text.lower() == 'q':
                break
            
            target_x, target_y = navigator.ask_llm_for_coordinates(user_text)
            if target_x is not None and target_y is not None:
                navigator.send_navigation_goal(target_x, target_y)
                
            # 维持 ROS 2 节点的存活
            rclpy.spin_once(navigator, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
        
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
