#!/usr/bin/env python3
"""导航：AWS 世界 + TurtleBot3 + Nav2 (AMCL 定位 + 地图)。

用法示例:
    ros2 launch aws_world_nav navigation.launch.py world:=house map:=/path/to/map.yaml
    ros2 launch aws_world_nav navigation.launch.py world:=warehouse \
        map:=~/Documents/Trae-CN/default/aws_nav_ws/maps/warehouse.yaml gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 与建图时一致的出生点，AMCL 初始位姿也默认设为该值（可通过 x/y/yaw 覆盖）
WORLD_SPAWN = {
    'house': {'x': '0.0', 'y': '0.0', 'yaw': '0.0'},
    'warehouse': {'x': '0.0', 'y': '8.0', 'yaw': '0.0'},
}


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
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

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': os.path.join(
                get_package_share_directory('nav2_bringup'),
                'params', 'nav2_params.yaml'),
        }.items(),
    )

    return [gazebo_cmd, bringup_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house'),
        DeclareLaunchArgument('map', description='保存的地图 yaml 完整路径'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
