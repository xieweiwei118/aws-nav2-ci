#!/usr/bin/env python3
"""Gazebo 加载 AWS RoboMaker 仿真世界并生成 TurtleBot3 Waffle。

用法示例:
    ros2 launch aws_world_nav aws_world.launch.py world:=house
    ros2 launch aws_world_nav aws_world.launch.py world:=warehouse gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 工作空间 src 目录（AWS 世界仓库所在位置）
# 优先级: worlds_base 启动参数 > AWS_WORLDS_BASE 环境变量 > 本机默认路径
DEFAULT_WORLDS_BASE = os.environ.get(
    'AWS_WORLDS_BASE',
    os.path.join(os.path.expanduser('~'), 'Documents', 'Trae-CN', 'default',
                 'aws_nav_ws', 'src'))

WORLD_FILES = {
    'house': ('aws-robomaker-small-house-world', 'worlds', 'small_house.world'),
    'warehouse': ('aws-robomaker-small-warehouse-world', 'worlds', 'small_warehouse.world'),
}


def _setup(context, *args, **kwargs):
    worlds_base = LaunchConfiguration('worlds_base').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x_pose = LaunchConfiguration('x').perform(context)
    y_pose = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_gui = LaunchConfiguration('gui').perform(context)

    if world not in WORLD_FILES:
        raise RuntimeError(f"未知世界 '{world}'，可选: {list(WORLD_FILES)}")
    world_file = os.path.join(worlds_base, *WORLD_FILES[world])
    if not os.path.isfile(world_file):
        raise RuntimeError(f'世界文件不存在: {world_file}')

    # 各世界仓库的 models 目录加入 GAZEBO_MODEL_PATH（保留原有路径）
    model_dirs = [os.path.join(worlds_base, repo, 'models')
                  for repo, _ in WORLD_FILES.values()]
    model_dirs.append(os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models'))
    gazebo_model_path = os.pathsep.join(model_dirs) + os.pathsep + \
        os.environ.get('GAZEBO_MODEL_PATH', '')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(use_gui),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw],
        output='screen',
    )

    return [set_model_path, gzserver_cmd, gzclient_cmd,
            robot_state_publisher_cmd, spawn_entity_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house',
                              description='house 或 warehouse'),
        DeclareLaunchArgument('worlds_base', default_value=DEFAULT_WORLDS_BASE,
                              description='AWS 世界仓库所在目录'),
        DeclareLaunchArgument('x', default_value='0.0', description='出生点 x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='出生点 y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出生朝向 (rad)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 客户端界面'),
        OpaqueFunction(function=_setup),
    ])
