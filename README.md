## 流程如下：
1. 启动程序后，planner 订阅 /odom 话题获取小车实时位置作为路径规划起始点；用户在 RVIZ 中指定目标点后，RVIZ 将目标点消息通过 /goal_pose 话题发布，planner 订阅该话题获取目标点，将起始点和目标点（转换为地图坐标系后）传入 A * 算法。
2. 当 A * 算法搜索到全局路径后，planner 将路径信息通过 /path 话题发布；tracker 订阅 /path 话题缓存全局路径，同时实时订阅 /odom 话题获取小车位姿，在全局路径中找到符合预瞄距离的目标点，通过路径跟踪算法（如纯追踪 / Stanley）计算速度指令，最终发布到 /cmd_vel 话题控制小车运动。

---

## 一、规划
### rover_planner_node
#### 订阅话题
- "/map"
    订阅 nav_msgs/OccupancyGrid 消息，提取地图的分辨率、宽度、高度、原点坐标等信息，解析栅格障碍数据，用于 A * 算法中初始化栅格地图。
- "/odom"
    订阅 nav_msgs/Odometry 消息，提取小车在 odom 坐标系下的位置（x/y）和航向角（yaw），经 TF 转换为 map 坐标系后，作为 A * 算法的起始点（实时更新）。
- "/goal_pose"
    订阅 geometry_msgs/PoseStamped 消息，提取目标点在 map 坐标系下的位置，校验目标点是否在可行区域（非障碍），用于 A * 算法中初始化路径的目标点。
#### 发布话题
- "/path"
    发布 nav_msgs/Path 消息，包含 A * 搜索到的全局路径点序列（map 坐标系），路径点间距可配置，发布前校验路径有效性（非空、无碰撞）。
    
---

## 二、跟踪
### path_tracking_node
#### 订阅话题
- "/path"
    订阅 nav_msgs/Path 消息，缓存 A * 搜索到的全局路径（覆盖旧路径），并转换为 odom 坐标系便于计算相对位置。
- "/odom"
    订阅 nav_msgs/Odometry 消息，接收小车的实时里程计数据，解析当前位置（x/y）和航向角（yaw），在全局路径中找到符合预瞄距离的目标点，通过跟踪算法计算速度指令。
#### 发布话题
- "/cmd_vel"
    发布 geometry_msgs/Twist 消息，包含小车的线速度（linear.x）和角速度（angular.z），发布前对速度做限幅（不超过小车物理极限）；无有效路径 / 到达终点时发布零速度。