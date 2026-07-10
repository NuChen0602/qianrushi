set(BOARD_APP_SOURCES
    main.cpp
    control/pid.cpp
    hardware/lidar_scanner.cpp
    hardware/robot_hardware.cpp
    utils/config.cpp
    utils/logger.cpp
)

add_executable(robot_board_app ${BOARD_APP_SOURCES})

set(LDLIDAR_ROOT "${CMAKE_SOURCE_DIR}/third_party/ldlidar_stl_sdk")
target_sources(robot_board_app PRIVATE
    ${LDLIDAR_ROOT}/ldlidar_driver/src/core/ldlidar_driver.cpp
    ${LDLIDAR_ROOT}/ldlidar_driver/src/dataprocess/lipkg.cpp
    ${LDLIDAR_ROOT}/ldlidar_driver/src/filter/tofbf.cpp
    ${LDLIDAR_ROOT}/ldlidar_driver/src/logger/log_module.cpp
    ${LDLIDAR_ROOT}/ldlidar_driver/src/networkcom/network_socket_interface_linux.cpp
    ${LDLIDAR_ROOT}/ldlidar_driver/src/serialcom/serial_interface_linux.cpp
)

target_include_directories(robot_board_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LDLIDAR_ROOT}/ldlidar_driver/include/core
    ${LDLIDAR_ROOT}/ldlidar_driver/include/dataprocess
    ${LDLIDAR_ROOT}/ldlidar_driver/include/filter
    ${LDLIDAR_ROOT}/ldlidar_driver/include/logger
    ${LDLIDAR_ROOT}/ldlidar_driver/include/networkcom
    ${LDLIDAR_ROOT}/ldlidar_driver/include/serialcom
)

find_package(Threads REQUIRED)
target_link_libraries(robot_board_app PRIVATE Threads::Threads)

if(ROBOT_USE_LS2K0301_LIBRARY)
    set(LS2K0301_LIBRARY_ROOT
        "${CMAKE_SOURCE_DIR}/third_party/LS2K0301_Library/LS2K030x_Library/Seekfree_LS2K030x_Opensource_Library"
        CACHE PATH "Path to SeekFree LS2K0301 library root"
    )
    set(LS2K0301_LIBRARIES_DIR "${LS2K0301_LIBRARY_ROOT}/libraries")

    target_compile_definitions(robot_board_app PRIVATE ROBOT_USE_LS2K0301_LIBRARY=1)
    target_include_directories(robot_board_app PRIVATE
        ${LS2K0301_LIBRARY_ROOT}
        ${LS2K0301_LIBRARIES_DIR}/zf_common
        ${LS2K0301_LIBRARIES_DIR}/zf_driver
        ${LS2K0301_LIBRARIES_DIR}/zf_device
        ${LS2K0301_LIBRARIES_DIR}/zf_components/seekfree_assistant
    )
    target_sources(robot_board_app PRIVATE
        ${LS2K0301_LIBRARIES_DIR}/zf_driver/zf_driver_file_buffer.cpp
        ${LS2K0301_LIBRARIES_DIR}/zf_driver/zf_driver_file_string.cpp
        ${LS2K0301_LIBRARIES_DIR}/zf_driver/zf_driver_encoder.cpp
        ${LS2K0301_LIBRARIES_DIR}/zf_driver/zf_driver_gpio.cpp
        ${LS2K0301_LIBRARIES_DIR}/zf_driver/zf_driver_pwm.cpp
        ${LS2K0301_LIBRARIES_DIR}/zf_device/zf_device_imu.cpp
    )
else()
    target_compile_definitions(robot_board_app PRIVATE ROBOT_USE_LS2K0301_LIBRARY=0)
endif()

target_compile_options(robot_board_app PRIVATE -Wall -Wextra -Wpedantic)
