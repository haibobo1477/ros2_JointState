#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # 订阅 JointState 消息
        self.subscription = self.create_subscription(
            JointState,
            'vx300s/joint_states',
            self.listener_callback,
            10)
        
        # 定时器：每隔 1 秒调用一次 timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Subscribed to vx300s/joint_states topic.')

        # 缓存最近一次的 joint state 消息
        self.latest_joint_state = None

    def listener_callback(self, msg):
        self.latest_joint_state = msg  # 存储最新的 joint state

    def timer_callback(self):
        if self.latest_joint_state:
            self.get_logger().info('Timer Triggered - Latest Joint States:')
            for name, position in zip(self.latest_joint_state.name, self.latest_joint_state.position):
                self.get_logger().info(f'  {name}: {position:.4f}')
        else:
            self.get_logger().info('Waiting for first joint state message...')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
