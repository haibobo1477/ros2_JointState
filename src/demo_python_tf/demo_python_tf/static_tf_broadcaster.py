import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros

class PoseListener(Node):
    def __init__(self):
        super().__init__('ee_pose_tf_listener')

        # 创建 TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 设置定时器，每 0.1 秒检查一次 TF
        self.timer = self.create_timer(0.1, self.lookup_transform)

    def lookup_transform(self):
        try:
            # 查找从 base_link 到 ee_gripper_link 的变换
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'vx300s/base_link',              # 父坐标系
                'vx300s/ee_gripper_link',        # 子坐标系
                rclpy.time.Time()                # 最新时间
            )

            # 获取平移位置
            t = transform.transform.translation
            self.get_logger().info(
                f"EE Pose → x: {t.x:.3f}, y: {t.y:.3f}, z: {t.z:.3f}"
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform unavailable: {e}")

def main():
    rclpy.init()
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



















# import rclpy
# from rclpy.node import Node
# from tf2_ros import StaticTransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from tf_transformations import quaternion_from_euler
# import math



# class StaticTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('Static_tf_broadcaster')
#         self.static_broadcaster_ = StaticTransformBroadcaster(self)
#         self.publish_static_tf()
        
        
        
#     def publish_static_tf(self):
#         """
#         发布静态TF, 从base_link到camera_link之间的坐标关系
#         """
#         transform = TransformStamped()
#         transform.header.frame_id = 'base_link'
#         transform.child_frame_id = 'camera_link'
#         transform.header.stamp = self.get_clock().now().to_msg()
        
        
#         transform.transform.translation.x = 0.5
#         transform.transform.translation.y = 0.3
#         transform.transform.translation.z = 0.6
        
#         # 欧拉角转四元数
#         q = quaternion_from_euler(math.radians(180),0,0)  # q是元组
#         transform.transform.rotation.x = q[0]
#         transform.transform.rotation.y = q[1]
#         transform.transform.rotation.z = q[2]
#         transform.transform.rotation.w = q[3]
        
#         self.static_broadcaster_.sendTransform(transform)
#         self.get_logger().info(f'发布静态TF:{transform}')
        
        


# def main():
#     rclpy.init()
#     node = StaticTFBroadcaster()
#     rclpy.spin(node)
#     rclpy.shutdown() 
        



