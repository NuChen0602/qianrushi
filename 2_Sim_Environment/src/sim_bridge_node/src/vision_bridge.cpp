#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <sstream>

// 包含我们刚刚手写的纯 C++ 算法核心头文件！
#include "VisionCore.h" 

class VisionBridgeNode : public rclcpp::Node {
public:
    VisionBridgeNode() : Node("vision_bridge_node") {
        // 实例化视觉大脑
        vision_core_ = std::make_shared<VisionCore>();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/low_angle_camera/image_raw", 10, 
            std::bind(&VisionBridgeNode::image_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&VisionBridgeNode::odom_callback, this, std::placeholders::_1));

        event_publisher_ = this->create_publisher<std_msgs::msg::String>("/patrol/events", 10);
        
        RCLCPP_INFO(this->get_logger(), "🧠 C++ 视觉桥接节点已启动！成功挂载 libvision_core.so");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;
        has_pose_ = true;
    }

    bool cooldown_ready(PatrolResult result) {
        const auto now = this->now();
        if (result == HAZARD_RED_SOCKET) {
            if ((now - last_hazard_publish_).seconds() < 3.0) return false;
            last_hazard_publish_ = now;
            return true;
        }
        if (result == LOST_BLUE_CARD) {
            if ((now - last_lost_publish_).seconds() < 3.0) return false;
            last_lost_publish_ = now;
            return true;
        }
        return false;
    }

    void publish_event(PatrolResult result) {
        if (!cooldown_ready(result)) return;

        std::ostringstream json;
        if (result == HAZARD_RED_SOCKET) {
            json << "{\"type\":\"safety_hazard\","
                 << "\"severity\":\"high\","
                 << "\"message\":\"发现疑似违规接线板/异常热源标记\",";
        } else if (result == LOST_BLUE_CARD) {
            json << "{\"type\":\"lost_found_ticket\","
                 << "\"severity\":\"info\","
                 << "\"message\":\"发现疑似校园卡，已生成失物招领工单\",";
        } else {
            return;
        }

        json << "\"source\":\"vision_core\","
             << "\"x\":" << (has_pose_ ? last_x_ : 0.0) << ","
             << "\"y\":" << (has_pose_ ? last_y_ : 0.0) << "}";

        std_msgs::msg::String msg;
        msg.data = json.str();
        event_publisher_->publish(msg);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat output_frame;

            // 核心调用：把 ROS 图像喂给你的跨平台大脑！
            PatrolResult result = vision_core_->processFrame(frame, output_frame);

            // 根据返回的枚举值执行业务逻辑
            if (result == HAZARD_RED_SOCKET) {
                RCLCPP_WARN(this->get_logger(), "🚨 [C++ 核心报警] 发现违规发热插座！");
                publish_event(result);
            } else if (result == LOST_BLUE_CARD) {
                RCLCPP_INFO(this->get_logger(), "📘 [C++ 核心报警] 发现疑似校园卡！");
                publish_event(result);
            }

            // 显示打上标签的画面
            cv::imshow("C++ Core Patrol View", output_frame);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
    std::shared_ptr<VisionCore> vision_core_; // 算法指针
    bool has_pose_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    rclcpp::Time last_hazard_publish_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lost_publish_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
