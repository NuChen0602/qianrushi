cmake_minimum_required(VERSION 3.10)
project(square_path_voice_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O2 -Wall -Wextra)

set(SEEKFREE_LIBRARY_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../LS2K0301_Library/LS2K030x_Library/Example/Motherboard_Demo/E3_motor/libraries")

add_executable(square_path_voice_test
    src/square_path_voice_test.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_gpio.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_pwm.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_encoder.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_file_buffer.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_driver/zf_driver_file_string.cpp
    ${SEEKFREE_LIBRARY_DIR}/zf_device/zf_device_imu.cpp
)

target_include_directories(square_path_voice_test PRIVATE
    ${SEEKFREE_LIBRARY_DIR}/zf_common
    ${SEEKFREE_LIBRARY_DIR}/zf_driver
    ${SEEKFREE_LIBRARY_DIR}/zf_device
)

target_link_libraries(square_path_voice_test PRIVATE pthread)
