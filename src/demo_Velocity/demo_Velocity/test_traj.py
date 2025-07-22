#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
from math import sin, pi

class DanceTestNode(Node):
    def __init__(self):
        super().__init__('dance_test_node')

        # 发布器：向 /vx300s/commands/joint_group 发送速度命令
        self.publisher = self.create_publisher(JointGroupCommand, '/vx300s/commands/joint_group', 1)

        # 初始时间戳
        self.ti = self.get_clock().now().nanoseconds / 1e9  # 秒

        # 定时器：每 0.1 秒调用一次回调
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        t_now = self.get_clock().now().nanoseconds / 1e9  # 当前时间（秒）
        elapsed = t_now - self.ti

        a_msg = JointGroupCommand()
        a_msg.name = 'arm'  # group name 要与你的 robot yaml 匹配

        if elapsed <= 1:
            a_msg.cmd = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]
        elif 1 < elapsed <= 3:
            a_msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif 3 < elapsed <= 10:
            # 摇摆动作（正弦波控制第三个关节）
            oscillate = sin(2 * pi * t_now)
            a_msg.cmd = [0.3, 0.0, oscillate, 0.0, 0.0, 0.0]
        else:
            a_msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher.publish(a_msg)
        self.get_logger().info(f"Published joint velocities: {a_msg.cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = DanceTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
