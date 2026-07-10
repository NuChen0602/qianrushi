cmake_minimum_required(VERSION 3.16)

project(robot_2k0301 VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOT_USE_LS2K0301_LIBRARY "Build with SeekFree LS2K0301 official library" OFF)

add_subdirectory(board_app)

