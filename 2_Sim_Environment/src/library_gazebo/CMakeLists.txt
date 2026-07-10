cmake_minimum_required(VERSION 3.8)
project(library_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装 launch, urdf 和 worlds 文件夹到 install 目录
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/llm_navigator.py
  scripts/llm_voice_navigator.py
  scripts/patrol_dashboard.py
  scripts/patrol_knowledge.py
  scripts/patrol_mission.py
  scripts/patrol_web_dashboard.py
  scripts/real_camera_publisher.py
  scripts/real_goal_driver.py
  scripts/real_motion_bridge.py
  scripts/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
