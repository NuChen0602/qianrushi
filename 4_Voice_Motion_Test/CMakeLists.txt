cmake_minimum_required(VERSION 3.10)
project(voice_motion_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O2 -Wall -Wextra)

set(SEEKFREE_LIBRARY_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../LS2K0301_Library/LS2K030x_Library/Example/Motherboard_Demo/E3_motor/libraries")

add_executable(voice_motion_test
    src/voice_motion_test.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_gpio.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_pwm.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_encoder.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_file_buffer.cpp
)

target_include_directories(voice_motion_test PRIVATE
    ${SEEKFREE_LIBRARY_DIR}/zf_common
    ${SEEKFREE_LIBRARY_DIR}/zf_driver
)

target_link_libraries(voice_motion_test PRIVATE pthread)
