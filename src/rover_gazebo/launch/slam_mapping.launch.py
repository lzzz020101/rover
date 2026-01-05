import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('rover_gazebo')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 1. 启动仿真环境 (车 + Gazebo)
    # 注意：这里调用你之前的 gazebo_sim.launch.py
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_sim.launch.py')
        )
    )

    # 2. 启动 SLAM Toolbox (异步建图模式)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true', # 关键！仿真必须开这个
            'slam_params_file': os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml') # 用默认配置先跑
        }.items()
    )

    return LaunchDescription([
        gazebo_sim,
        slam
    ])