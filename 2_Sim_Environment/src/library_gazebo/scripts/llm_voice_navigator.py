#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from openai import OpenAI
import httpx
import json
import os
import shutil
import subprocess

from patrol_knowledge import load_library_map, lookup_semantic_target, patrol_route_targets

try:
    import pygame
except ImportError:
    pygame = None

try:
    import speech_recognition as sr
except ImportError:
    sr = None

class LLMVoiceNavigatorNode(Node):
    def __init__(self):
        super().__init__('llm_voice_navigator_node')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/patrol/mission_status', 10)
        self.real_motion_pub = self.create_publisher(String, '/patrol/real_motion_cmd', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("real_goal_wait_sec", 8.0)
        self.declare_parameter("direct_forward_counts", 30)
        self.declare_parameter("direct_turn_counts", 16)
        self.use_nav2 = bool(self.get_parameter("use_nav2").value)
        self.real_goal_wait_sec = float(self.get_parameter("real_goal_wait_sec").value)
        self.direct_forward_counts = int(self.get_parameter("direct_forward_counts").value)
        self.direct_turn_counts = int(self.get_parameter("direct_turn_counts").value)
        
        # 初始化语音播放模块
        self.voice_enabled = pygame is not None and shutil.which("edge-tts") is not None
        if pygame is None:
            self.get_logger().warn("未安装 pygame，语音播报降级为终端文本。")
        elif shutil.which("edge-tts") is None:
            self.get_logger().warn("未找到 edge-tts，语音播报降级为终端文本。")
        else:
            try:
                pygame.mixer.init()
            except Exception as e:
                self.voice_enabled = False
                self.get_logger().warn(f"音频设备初始化失败，语音播报降级为终端文本: {e}")

        self.speech_recognition_enabled = sr is not None
        if sr is None:
            self.get_logger().warn("未安装 speech_recognition，语音输入降级为键盘输入。")
        
        # 初始化 DeepSeek 客户端
        self.api_key = os.getenv("DEEPSEEK_API_KEY")
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
        self.patrol_route = patrol_route_targets(self.library_map)
        self.wake_words = ["机器人", "小车", "巡检机器人", "你好小车", "你好机器人"]
        self.get_logger().info('🤖 具身智能语音交互节点已启动！')
        # 启动时先打个招呼
        self.speak("系统已启动，我是智能巡检机器人。需要服务时，请先说机器人或小车唤醒我。")

    def speak(self, text):
        """调用 Edge TTS 生成语音并播放 (嘴巴)"""
        self.get_logger().info(f"🔊 机器人说: {text}")
        if not self.voice_enabled:
            return

        audio_file = "/tmp/robot_speech.mp3"
        try:
            # 使用 zh-CN-XiaoxiaoNeural (微软晓晓，极其自然的女声)
            subprocess.run(
                ["edge-tts", "--voice", "zh-CN-XiaoxiaoNeural", "--text", text, "--write-media", audio_file],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True
            )
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        except Exception as e:
            self.get_logger().error(f"语音合成失败: {e}")

    def listen_phrase(self, prompt, timeout=5, phrase_time_limit=8, warn_on_timeout=False):
        """调用麦克风获取一句语音并转文字。超时只返回 None，不切键盘。"""
        if not self.speech_recognition_enabled:
            return None

        r = sr.Recognizer()
        try:
            with sr.Microphone() as source:
                self.get_logger().info(prompt)
                # 自动适应环境噪声
                r.adjust_for_ambient_noise(source, duration=0.5)
                audio = r.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
            self.get_logger().info("⏳ 正在识别语音...")
            text = r.recognize_google(audio, language='zh-CN')
            self.get_logger().info(f"✅ 听懂了: {text}")
            return text
        except sr.WaitTimeoutError:
            if warn_on_timeout:
                self.get_logger().warn("这次没有听到声音，继续待机。")
        except sr.UnknownValueError:
            self.get_logger().info("这句话没有识别清楚，继续待机。")
        except sr.RequestError as e:
            self.speech_recognition_enabled = False
            self.get_logger().warn(f"云端语音识别不可用，降级为键盘输入: {e}")
        except OSError as e:
            self.speech_recognition_enabled = False
            self.get_logger().warn(f"麦克风不可用，降级为键盘输入: {e}")
        except Exception as e:
            self.get_logger().warn(f"语音识别临时失败，继续待机: {e}")
        return None

    def wait_for_wake_word(self):
        """常驻待机监听唤醒词。"""
        self.get_logger().info(f"待机中，请说唤醒词：{' / '.join(self.wake_words)}")
        while rclpy.ok() and self.speech_recognition_enabled:
            text = self.listen_phrase(
                "\n👂 待机监听中：请先说“机器人”或“小车”唤醒我。",
                timeout=4,
                phrase_time_limit=4,
                warn_on_timeout=False
            )
            rclpy.spin_once(self, timeout_sec=0.05)
            if not text:
                continue
            if "键盘" in text:
                return "keyboard"
            if any(word in text for word in self.wake_words):
                self.get_logger().info("唤醒成功。")
                return "voice"
            self.get_logger().info("未检测到唤醒词，继续待机。")
        return "keyboard"

    def listen_for_command(self):
        """唤醒后监听一次业务指令。"""
        self.speak("我在，请说出您的需求。")
        return self.listen_phrase(
            "\n👂 已唤醒，请说导航或巡检指令。",
            timeout=8,
            phrase_time_limit=12,
            warn_on_timeout=True
        )

    def ask_llm_for_coordinates_and_speech(self, user_input):
        """让大模型同时思考坐标和回复的话 (大脑)"""
        self.get_logger().info(f'🤔 大脑正在思考: "{user_input}" ...')
        
        system_prompt = f"""你是一个友好的图书馆智能巡检机器人。
        你的任务是根据用户的语言请求，结合[馆藏与语义地图]推理出目标坐标。
        {self.library_map_prompt}
        请输出一段安抚用户或指引的简短回复（speech字段），以及目标坐标（x和y字段）。
        如果用户的话没有明确目的地，x和y请设为 0.0。
        请严格输出JSON格式，不要包含任何其他文字。
        格式示例：{{"speech": "好的，我已经知道您要找视觉算法的书了，马上带您前往A区。", "x": 2.5, "y": 1.0}}"""

        try:
            if self.client is None:
                raise RuntimeError("DEEPSEEK_API_KEY is not set")
            
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_input}
                ],
                response_format={"type": "json_object"}
            )
            
            result_json = json.loads(response.choices[0].message.content)
            return result_json.get('speech', "好的，马上出发。"), result_json.get('x', 0.0), result_json.get('y', 0.0)
            
        except Exception as e:
            self.get_logger().error(f"调用大脑失败: {e}")
            return self.local_brain(user_input)

    def local_brain(self, user_input):
        """网络或 API 不可用时的本地关键词兜底。"""
        target = lookup_semantic_target(user_input, self.library_map)
        return target.speech, target.x, target.y

    def send_navigation_goal(self, x, y):
        """向底层发送控制指令 (脊髓)"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0 
        
        self.goal_pub.publish(goal_msg)

        if not self.use_nav2:
            self.publish_mission_status("real_goal", f"实机模式已下发目标点：x={float(x):.2f}, y={float(y):.2f}")
            self.get_logger().info("实机模式未调用 Nav2，目标将由 real_goal_driver 转换为底盘动作。")
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("⚠️ Nav2 未就绪。")
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
            self.get_logger().error("Nav2 拒绝了目标点，请检查 TF、costmap 或目标点是否可达。")
            return

        self.get_logger().info("Nav2 已接受语音导航目标。")

    def is_patrol_command(self, user_input):
        keywords = ["开始巡检", "执行巡检", "自动巡检", "巡检一遍", "低视角扫描", "扫描失物", "检查插座"]
        return any(keyword in user_input for keyword in keywords)

    def publish_real_motion(self, command):
        msg = String()
        msg.data = command
        self.real_motion_pub.publish(msg)
        self.publish_mission_status("voice_direct_motion", command)
        self.get_logger().info(f"实机动作指令: {command}")

    def handle_direct_robot_command(self, user_input):
        """优先处理实机短动作，不交给 LLM 猜坐标。"""
        text = user_input.replace(" ", "").lower()

        if any(word in text for word in ["急停", "停止", "停车", "停下", "别动", "stop"]):
            self.publish_real_motion("STOP")
            self.speak("已停车。")
            return True

        if any(word in text for word in ["舵机回正", "方向回正", "车轮回正", "轮子回正", "回正"]):
            self.publish_real_motion("SERVO center")
            self.speak("舵机已回正。")
            return True

        if any(word in text for word in ["舵机左", "方向左", "车轮左", "左打"]):
            self.publish_real_motion("SERVO left")
            self.speak("前轮向左。")
            return True

        if any(word in text for word in ["舵机右", "方向右", "车轮右", "右打"]):
            self.publish_real_motion("SERVO right")
            self.speak("前轮向右。")
            return True

        if any(word in text for word in ["直线前进", "直行", "向前走", "往前走", "前进"]):
            self.publish_real_motion("SERVO center")
            self.publish_real_motion(f"MOVE forward {self.direct_forward_counts}")
            self.speak("收到，直线前进。")
            return True

        if any(word in text for word in ["直线后退", "向后退", "往后退", "后退", "倒车"]):
            self.publish_real_motion("SERVO center")
            self.publish_real_motion(f"MOVE back {self.direct_forward_counts}")
            self.speak("收到，直线后退。")
            return True

        if "左转" in text:
            self.publish_real_motion(f"TURN left {self.direct_turn_counts}")
            self.speak("收到，左转。")
            return True

        if "右转" in text:
            self.publish_real_motion(f"TURN right {self.direct_turn_counts}")
            self.speak("收到，右转。")
            return True

        return False

    def publish_mission_status(self, event_type, message, target=None):
        payload = {"type": event_type, "message": message}
        if target is not None:
            payload.update({"area_id": target.area_id, "x": target.x, "y": target.y})
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def run_voice_patrol_route(self):
        """语音触发的巡检路线：依次前往语义地图中的巡检点。"""
        if not self.patrol_route:
            self.speak("当前没有配置巡检路线。")
            return

        if not self.use_nav2:
            self.publish_mission_status("mission_start", "实机模式执行语义巡检路线。")
            self.speak("收到，开始执行实机巡检。")
            for target in self.patrol_route:
                if not rclpy.ok():
                    break
                self.publish_mission_status("mission_waypoint", target.speech, target)
                self.speak(target.speech)
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.position.x = float(target.x)
                goal_msg.pose.position.y = float(target.y)
                goal_msg.pose.position.z = 0.0
                goal_msg.pose.orientation.w = 1.0
                self.goal_pub.publish(goal_msg)
                end_time = self.get_clock().now().nanoseconds / 1e9 + max(1.0, self.real_goal_wait_sec)
                while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < end_time:
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.publish_mission_status("mission_arrived", f"已到达{target.name}附近，正在执行低视角扫描。", target)
            self.publish_mission_status("mission_done", "实机语义巡检路线执行完成。")
            self.speak("本轮实机巡检已完成。")
            return

        if not self.nav_to_pose_client.server_is_ready():
            self.publish_mission_status("mission_waiting", "收到巡检指令，正在等待 Nav2 导航系统就绪。")
            self.speak("收到巡检指令，导航系统正在准备，请稍等。")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=60.0):
            self.publish_mission_status("mission_error", "Nav2 未就绪，无法启动语音巡检。")
            self.speak("导航系统还没有准备好，暂时不能开始巡检。")
            return

        self.publish_mission_status("mission_start", "语音指令触发自动巡检路线。")
        self.speak("收到，开始执行图书馆低视角巡检。")

        for target in self.patrol_route:
            if not rclpy.ok():
                break

            self.publish_mission_status("mission_waypoint", target.speech, target)
            self.speak(target.speech)

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = float(target.x)
            goal_msg.pose.position.y = float(target.y)
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)

            goal = NavigateToPose.Goal()
            goal.pose = goal_msg
            send_future = self.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

            if not send_future.done() or send_future.result() is None:
                self.publish_mission_status("mission_error", f"发送巡检目标超时：{target.name}", target)
                continue

            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.publish_mission_status("mission_error", f"Nav2 拒绝巡检目标：{target.name}", target)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.publish_mission_status("mission_arrived", f"已到达{target.name}，正在执行低视角扫描。", target)

        self.publish_mission_status("mission_done", "语音触发的巡检路线执行完成。")
        self.speak("本轮巡检已完成。")

def main(args=None):
    rclpy.init(args=args)
    navigator = LLMVoiceNavigatorNode()
    
    try:
        while rclpy.ok():
            if navigator.speech_recognition_enabled:
                mode = navigator.wait_for_wake_word()
                if mode == "voice":
                    user_text = navigator.listen_for_command()
                    if not user_text:
                        navigator.get_logger().info("未收到有效指令，回到待机状态。")
                        continue
                else:
                    user_text = None
            else:
                user_text = None

            if not user_text:
                user_text = input("\n⌨️ 键盘模式，请手动输入指令 (输入 q 退出): ")
                if user_text.lower() == 'q': break
                if not user_text.strip(): continue

            if navigator.handle_direct_robot_command(user_text):
                rclpy.spin_once(navigator, timeout_sec=0.1)
                continue

            if navigator.is_patrol_command(user_text):
                navigator.run_voice_patrol_route()
                rclpy.spin_once(navigator, timeout_sec=0.1)
                continue

            # 2. 想
            speech_text, target_x, target_y = navigator.ask_llm_for_coordinates_and_speech(user_text)
            
            if speech_text:
                # 3. 说
                navigator.speak(speech_text)
                # 4. 做
                if target_x != 0.0 or target_y != 0.0:
                    navigator.send_navigation_goal(target_x, target_y)
                    
            rclpy.spin_once(navigator, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
        
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
