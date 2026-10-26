#!/usr/bin/env python3
"""反应式自动探索节点：读取 /scan，发布 /cmd_vel，驱动 TurtleBot3 游走建图。

策略：
- 前方开阔时直行，并根据左右空旷程度微调方向（远离障碍）；
- 前方过近时原地转向更空旷的一侧；
- 四周都被堵住时缓慢自旋寻找出路；
- 运行 duration（仿真秒）后停车退出，便于后续保存地图。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutoExplorer(Node):

    def __init__(self):
        super().__init__('auto_explorer')
        self.declare_parameter('duration', 300.0)       # 运行时长（仿真秒）
        self.declare_parameter('forward_speed', 0.18)   # 直行速度 m/s
        self.declare_parameter('turn_speed', 0.55)      # 原地转向 rad/s
        self.declare_parameter('safe_dist', 0.45)       # 触发转向的前方安全距离 m
        self.declare_parameter('hard_dist', 0.30)       # 紧急距离 m

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.last_scan = None
        self.start_time = self.now()
        self.duration = float(
            self.get_parameter('duration').get_parameter_value().double_value)

    def now(self):
        return self.get_clock().now()

    def elapsed(self):
        return (self.now() - self.start_time).nanoseconds * 1e-9

    def scan_cb(self, msg):
        self.last_scan = msg

    @staticmethod
    def _sector_min(ranges, a0, a1):
        """取 [a0, a1] 弧度扇区内的最小有效距离，无有效点返回 inf。"""
        best = float('inf')
        for r in ranges:
            if math.isfinite(r) and r > 0.05:
                best = min(best, r)
        return best

    def sector(self, msg, a0, a1):
        n = len(msg.ranges)
        i0 = max(0, int((a0 - msg.angle_min) / msg.angle_increment))
        i1 = min(n - 1, int((a1 - msg.angle_min) / msg.angle_increment))
        return self._sector_min(msg.ranges[i0:i1 + 1], a0, a1)

    def control_loop(self):
        t = self.elapsed()
        if t > self.duration:
            self.cmd_pub.publish(Twist())  # 停车
            self.get_logger().info('探索时间到，停止并退出。')
            raise SystemExit(0)

        msg = self.last_scan
        cmd = Twist()
        if msg is None:
            self.cmd_pub.publish(cmd)
            return

        fwd = self.sector(msg, -0.35, 0.35)     # 正前 ±20°
        left = self.sector(msg, 0.5, 1.4)       # 左前
        right = self.sector(msg, -1.4, -0.5)    # 右前
        around = self.sector(msg, -1.6, 1.6)    # 前方半圈

        fwd_speed = float(
            self.get_parameter('forward_speed').get_parameter_value().double_value)
        turn_speed = float(
            self.get_parameter('turn_speed').get_parameter_value().double_value)
        safe = float(self.get_parameter('safe_dist')
                     .get_parameter_value().double_value)
        hard = float(self.get_parameter('hard_dist')
                     .get_parameter_value().double_value)

        if around < hard:
            # 四周受困：缓慢自旋
            cmd.angular.z = 0.4 * turn_speed
        elif fwd < safe:
            # 前方过近：转向更空旷一侧
            cmd.angular.z = turn_speed if left >= right else -turn_speed
        else:
            # 直行 + 远离较近一侧
            cmd.linear.x = fwd_speed
            diff = right - left  # >0 左边更空 → 左转
            cmd.angular.z = max(-0.6, min(0.6, 0.8 * diff))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
