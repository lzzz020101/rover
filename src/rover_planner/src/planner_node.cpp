/*
功能：唯一与 ROS 2 交互的类。
核心逻辑：
订阅：/map（地图）、/goal_pose（RViz 给的目标点）、/odom（小车当前位置）。
逻辑控制：当收到目标点，先调 grid_map 更新环境，再调 astar 算路径，最后调 bspline 优化。
发布：发布可视化路径（visualization_msgs/Marker）给 RViz 观察，发布平滑路径给控制器。
*/