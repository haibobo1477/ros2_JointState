import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros

class PoseListener(Node):
    def __init__(self):
        super().__init__('ee_pose_tf_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # set timer, every 0.1s 
        self.timer = self.create_timer(0.1, self.lookup_transform)

    def lookup_transform(self):
        try:
            # check from base_link to ee_gripper_link 
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'vx300s/base_link',              # parent coordinate
                'vx300s/ee_gripper_link',        # child coordinate
                rclpy.time.Time()                # update time
            )

            # get translation and orientation
            t = transform.transform.translation
            r = transform.transform.rotation
            
            self.get_logger().info(
                f"EE Pose → x: {t.x:.3f}, y: {t.y:.3f}, z: {t.z:.3f},"
                f"qx: {r.x:.3f}, qy: {r.y:.3f}, qz: {r.z:.3f}, qw: {r.w:.3f}"
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
        



