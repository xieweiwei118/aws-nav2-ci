#!/usr/bin/env python3
"""建图：AWS 世界 + TurtleBot3 + slam_toolbox + 自动探索。

用法示例:
    ros2 launch aws_world_nav mapping.launch.py world:=house explore:=true explore_time:=300
    ros2 launch aws_world_nav mapping.launch.py world:=house explore:=false gui:=false
  (explore:=false 时只建图不动，可用键盘 teleop 手动遥控)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_SPAWN = {
    # 出生点选在世界中开阔、无障碍的位置
    'house': {'x': '0.0', 'y': '0.0', 'yaw': '0.0'},
    'warehouse': {'x': '0.0', 'y': '8.0', 'yaw': '0.0'},
}


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    explore = LaunchConfiguration('explore').perform(context)
    explore_time = LaunchConfiguration('explore_time').perform(context)

    spawn = WORLD_SPAWN.get(world, WORLD_SPAWN['house'])

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aws_world_nav'),
                         'launch', 'aws_world.launch.py')
        ),
        launch_arguments={
            'world': world,
            'x': spawn['x'], 'y': spawn['y'], 'yaw': spawn['yaw'],
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('aws_world_nav'),
                'config', 'slam_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    actions = [gazebo_cmd, slam_cmd]

    explorer_cmd = Node(
        package='aws_world_nav', executable='auto_explorer', name='auto_explorer',
        output='screen',
        parameters=[{'use_sim_time': True, 'duration': float(explore_time)}],
        condition=IfCondition(LaunchConfiguration('explore')),
    )
    actions.append(explorer_cmd)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house',
                              description='house 或 warehouse'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='启动反应式自动探索建图'),
        DeclareLaunchArgument('explore_time', default_value='300',
                              description='自动探索时长（仿真秒）'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 客户端界面'),
        OpaqueFunction(function=_setup),
    ])
