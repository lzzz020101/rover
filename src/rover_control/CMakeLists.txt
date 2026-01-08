cmake_minimum_required(VERSION 3.8)
project(rover_control)

# ============================================================
# 1. 编译选项
# ============================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 2. 查找依赖
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================
# 3. 头文件路径
# ============================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ============================================================
# 4. 库目标（先库，后节点）
# ============================================================
add_library(udp_comm_lib
  src/udp_communication.cpp
)

ament_target_dependencies(
  udp_comm_lib
  rclcpp
)

# 安装库
install(
  TARGETS udp_comm_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================
# 5. 可执行文件（ROS2 节点）
# ============================================================

# ---------- motion_control_node ----------
add_executable(motion_control_node
  src/motion_control_node.cpp
)

ament_target_dependencies(
  motion_control_node
  rclcpp
  geometry_msgs
)

target_link_libraries(
  motion_control_node
  udp_comm_lib
)

# ---------- path_tracking_node ----------
add_executable(path_tracking_node
  src/path_tracking_node.cpp
)

ament_target_dependencies(
  path_tracking_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

target_link_libraries(
  path_tracking_node
  udp_comm_lib
)

# ============================================================
# 6. 安装可执行文件
# ============================================================
install(
  TARGETS
    motion_control_node
    path_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# 7. 测试（永远靠后）
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================
# 8. ament 结尾（必须是最后一行）
# ============================================================
ament_package()
