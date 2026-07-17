cmake_minimum_required(VERSION 3.10)
project(aruco_board_stream LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

add_executable(board_stream_server board_stream_server.cpp)
target_include_directories(board_stream_server PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(board_stream_server PRIVATE ${OpenCV_LIBS})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(board_stream_server PRIVATE -Wall -Wextra -Wpedantic)
endif()
