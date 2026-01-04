cmake_minimum_required(VERSION 3.8)
project(rover_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # 虽然测试不用，但包可能需要
find_package(Eigen3 REQUIRED) # 必须有！

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 2. 编译 A* 核心库 (把你的 astar.cpp 编译成库，方便调用)
# 假设你的实现文件是 src/astar.cpp
add_library(astar_lib src/astar.cpp)
target_link_libraries(astar_lib Eigen3::Eigen)

# 3. 编译测试程序 (Test Node)
add_executable(test_astar src/test_astar.cpp)
target_link_libraries(test_astar 
  astar_lib 
  Eigen3::Eigen
)

# 安装指令 (可选，为了能在 install 里找到)
install(TARGETS 
  test_astar
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  astar_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()