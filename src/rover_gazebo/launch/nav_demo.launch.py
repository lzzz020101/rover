import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('rover_gazebo')
    pkg_planner = get_package_share_directory('rover_planner')
    pkg_control = get_package_share_directory('rover_control')

    # 1. 地图路径
    map_file = os.path.join(pkg_planner, 'maps', 'my_room.yaml')

    # 2. 启动仿真环境 (Gazebo + Robot State Publisher)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_sim.launch.py')
        )
    )

    # 3. 启动 Map Server (发布地图)
    # 使用 nav2_map_server 的 Lifecycle Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_file}]
    )

    # Map Server 需要一个 Lifecycle Manager 来激活
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 4. TF 胶水 (连接 map -> odom)
    # 假装定位是完美的 (因为 Gazebo 的 Odom 很准)
    tf_glue = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 5. 启动你的 Planner (大脑)
    planner_node = Node(
        package='rover_planner',
        executable='rover_planner_node',
        name='rover_planner_node',
        output='screen'
    )

    # 6. 启动你的 Tracker (小脑)
    tracker_node = Node(
        package='rover_control',
        executable='path_tracking_node',
        name='path_tracking_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        map_server,
        lifecycle_manager,
        tf_glue,
        planner_node,
        tracker_node
    ])