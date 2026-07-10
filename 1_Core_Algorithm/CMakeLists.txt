cmake_minimum_required(VERSION 3.10)
project(VisionCoreLib)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找系统上的 OpenCV
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(include ${OpenCV_INCLUDE_DIRS})

# 将算法源文件编译为共享动态库 (.so)
add_library(vision_core SHARED src/VisionCore.cpp)

# 链接 OpenCV 库
target_link_libraries(vision_core ${OpenCV_LIBS})