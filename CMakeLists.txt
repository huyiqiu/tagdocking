cmake_minimum_required(VERSION 3.8)
project(tagdocking)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ── Zsibot SDK bridge (C++) ───────────────────────────────────────
# 链接系统已装的 zsibot-sdk 静态库 (libzsibot.a) 及其运行库 libnetwork.so;
# 头文件在 /usr/local/include/zsibot/include (dpkg: zsibot-sdk, simple-socket)。
# SDK 不完整 (缺 libnetwork.so) 的机器自动跳过本目标: 备用 SDK 桥不用于
# cmd_vel 链路, 不应阻塞整个包的编译。
if(EXISTS /usr/local/lib/libzsibot.a AND EXISTS /usr/local/lib/libnetwork.so)
  add_executable(zsibot_bridge src/zsibot_bridge_node.cpp)
  target_include_directories(zsibot_bridge PRIVATE /usr/local/include/zsibot/include)
  target_link_libraries(zsibot_bridge
    /usr/local/lib/libzsibot.a
    /usr/local/lib/libnetwork.so
    pthread)
  ament_target_dependencies(zsibot_bridge rclcpp geometry_msgs nav_msgs std_srvs)
  install(TARGETS zsibot_bridge DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "zsibot SDK 不完整 (缺 libzsibot.a 或 libnetwork.so), 跳过 zsibot_bridge")
endif()

# ── Generate action interface ─────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  DEPENDENCIES action_msgs
)

# ── Install Python package ────────────────────────────────────────
# All modules under tagdocking/ (including base_adapter/)
install(
  DIRECTORY tagdocking
  DESTINATION local/lib/python3.10/dist-packages
  PATTERN "__pycache__" EXCLUDE
)

# ── Install entry-point scripts ───────────────────────────────────
install(PROGRAMS
  scripts/docking_node
  scripts/camera_info_bridge
  scripts/rtsp_camera
  scripts/calibrate_rtsp
  scripts/test_turn_angle
  scripts/test_jog_distance
  scripts/test_apriltag
  scripts/calibrate_camera
  scripts/mock_l1w_control
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch and config ─────────────────────────────────────
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(dual_docking_tests test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
               PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
