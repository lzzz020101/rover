cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

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
find_package(gazebo_ros_pkgs REQUIRED)

# ============================================================
# 3. 安装资源文件
# ============================================================
# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 worlds 文件夹
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 4. 测试
# ============================================================
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
# ============================================================
# 5. ament 结束
# ============================================================
ament_package()