cmake_minimum_required(VERSION 3.8)
project(rover_planner)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================
# 3. 头文件与资源安装
# ============================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 安装地图资源
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 4. 算法库
# ============================================================

# ---------- A* ----------
add_library(astar_lib
  src/astar.cpp
)

target_link_libraries(
  astar_lib
  Eigen3::Eigen
)

# ---------- B-spline ----------
add_library(bspline_lib
  src/bspline_optimizer.cpp
)

target_link_libraries(
  bspline_lib
  Eigen3::Eigen
)

# ============================================================
# 5. 规划器节点（ROS2 Node）
# ============================================================
add_executable(rover_planner_node
  src/rover_planner_node.cpp
)

ament_target_dependencies(
  rover_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(
  rover_planner_node
  astar_lib
  bspline_lib
  Eigen3::Eigen
)

# ============================================================
# 6. 安装目标
# ============================================================

# 安装节点
install(
  TARGETS rover_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库
install(
  TARGETS
    astar_lib
    bspline_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

# ============================================================
# 7. ament 结束
# ============================================================
ament_package()