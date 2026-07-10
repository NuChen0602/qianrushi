cmake_minimum_required(VERSION 3.8)
project(sim_bridge_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 魔法 1：引入外部的 C++ 大脑头文件目录
include_directories($ENV{HOME}/Library_Patrol_Project/1_Core_Algorithm/include)

add_executable(vision_bridge src/vision_bridge.cpp)

# 魔法 2：直接链接外部编译好的 libvision_core.so 动态库
target_link_libraries(vision_bridge
  $ENV{HOME}/Library_Patrol_Project/1_Core_Algorithm/build/libvision_core.so
)

ament_target_dependencies(vision_bridge rclcpp sensor_msgs std_msgs nav_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  vision_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
