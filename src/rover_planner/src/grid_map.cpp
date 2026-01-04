/*
grid_map (环境感知)
功能：把 ROS 2 的 /map 话题（nav_msgs/OccupancyGrid）转换成算法能读懂的 Eigen 矩阵。
核心逻辑：
坐标变换：世界坐标(x,y)到栅格索引(i,j)的转换。
膨胀逻辑：为了安全，把墙壁周围的格子也设为“障碍物”。
提供接口：给 A* 提供 isObstacle(x, y) 函数。
*/