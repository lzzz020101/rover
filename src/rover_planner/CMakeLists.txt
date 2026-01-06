cmake_minimum_required(VERSION 3.8)
project(rover_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖 (关键修正：补全了 nav_msgs 和 geometry_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)      
find_package(geometry_msgs REQUIRED) 

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# 2. 编译 A* 库
add_library(astar_lib src/astar.cpp)
target_link_libraries(astar_lib Eigen3::Eigen)

# 3. 编译 Bspline 库
add_library(bspline_lib src/bspline_optimizer.cpp)
target_link_libraries(bspline_lib Eigen3::Eigen)


# 4. 编译规划器节点 (ROS2 Node)
add_executable(rover_planner_node src/rover_planner_node.cpp)
ament_target_dependencies(rover_planner_node 
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

# 5. 安装指令
install(TARGETS 
  rover_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  astar_lib
  bspline_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()