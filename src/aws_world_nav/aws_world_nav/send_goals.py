#!/usr/bin/env python3
"""用 nav2_simple_commander 向 Nav2 发送一系列目标点，验证自主导航。

用法（在 navigation.launch.py 运行后）:
    ros2 run aws_world_nav send_goals --ros-args \
        -p goals:="[-3.0,1.0,0.0, 2.0,2.0,1.57]" -p initial_pose:="[0.0,0.0,0.0]"

goals: 扁平数组 [x1,y1,yaw1, x2,y2,yaw2, ...]，yaw 为弧度；
initial_pose: [x, y, yaw]，须与出生点一致。
"""

import math
import sys

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.declare_parameter('goals', [0.0, 0.0, 0.0])
    nav.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
    goals = list(nav.get_parameter('goals').get_parameter_value().double_array_value)
    init = list(nav.get_parameter('initial_pose').get_parameter_value().double_array_value)

    goal_list = [goals[i:i + 3] for i in range(0, len(goals) - 2, 3)]

    nav.setInitialPose(make_pose(nav, init[0], init[1], init[2]))
    nav.waitUntilNav2Active()

    if not goal_list:
        nav.get_logger().info('未提供有效 goals 参数，退出。')
        rclpy.shutdown()
        return

    succeeded = 0
    for i, (x, y, yaw) in enumerate(goal_list):
        nav.get_logger().info(
            f'>>> 目标 {i + 1}/{len(goal_list)}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        nav.goToPose(make_pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback is not None and hasattr(feedback, 'elapsed_time') and \
                    Duration.from_msg(feedback.elapsed_time) > Duration(seconds=180):
                nav.get_logger().warn('单个目标超过 180s，取消。')
                nav.cancelTask()
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            succeeded += 1
            nav.get_logger().info(f'目标 {i + 1} 到达成功')
        elif result == TaskResult.CANCELED:
            nav.get_logger().warn(f'目标 {i + 1} 被取消')
        else:
            nav.get_logger().error(f'目标 {i + 1} 失败')

    nav.get_logger().info(f'===== 导航测试完成: {succeeded}/{len(goal_list)} 个目标到达 =====')
    nav.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if succeeded == len(goal_list) else 1)


if __name__ == '__main__':
    main()
