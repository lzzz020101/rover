import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro
"""
1. os:负责拼接和处理文件路径
2. get_package_share_directory:通过包名，获取该 ROS 2 包的 share 目录的绝对路径
3. LaunchDescription:是一个容器，装所有要启动的东西
4. IncludeLaunchDescription：用来包含另一个launch文件
5. ExecuteProcess：直接执行一个系统进程，例如gazebo
6. RegisterEventHandler：注册事件监听器
7. PythonLaunchDescriptionSource：py类型的launch文件来源，通常和4. 连用
8. OnProcessExit：当某个进程 / 节点“退出”时，触发你指定的动作
9. Node：是 ROS 2 launch 中用于启动 ROS 节点的核心 Action。
10. import xacro 让 launch 文件能够在运行时把 .xacro 解析成 URDF
"""

def generate_launch_description():
    # 1. 定义包名和文件路径
    description_pkg_name = 'rover_description'
    gazebo_pkg_name = 'rover_gazebo'
    
    # 获取文件路径
    description_pkg_share = get_package_share_directory(description_pkg_name)
    gazebo_pkg_share = get_package_share_directory(gazebo_pkg_name)
    
    # 核心 xacro 文件位置
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'rover.xacro')
    
    # 2. 解析 Xacro 文件 (转成 URDF XML)
    # 返回一个对象，里面是完整的URDF XML树
    doc = xacro.process_file(xacro_file)
    # .toxml() 把 XML 树 转换成字符串
    robot_description_config = doc.toxml()
    
    # 3. 配置节点
    
    # Robot State Publisher: 发布 TF 树和 robot_description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # 键是规定名称，值是xacro转换出来的内容
        parameters=[{'robot_description': robot_description_config,
                     'use_sim_time': True}]
    )

    # Gazebo 启动项 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': os.path.join(gazebo_pkg_share,'worlds','my_room.world')}.items()
    )

    # Spawn Entity: 把机器人“生”在 Gazebo 里
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_rover'
                ],
        output='screen'
    )
    
    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}], 
        # arguments=['-d', ...], 
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz_node 
    ])