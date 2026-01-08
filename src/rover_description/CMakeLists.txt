cmake_minimum_required(VERSION 3.8)
project(rover_description)

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
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# ============================================================
# 3. 安装资源文件
# ============================================================

# 安装 URDF / Xacro
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装 mesh 文件
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 4. 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmakeopyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================
# 5. ament 结束
# ============================================================
ament_package()