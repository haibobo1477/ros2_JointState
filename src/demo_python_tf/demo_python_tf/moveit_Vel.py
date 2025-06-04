# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import RobotState
# from moveit_msgs.srv import GetCartesianPath
# from std_msgs.msg import Float64MultiArray

# # [class](https://wenku.csdn.net/doc/6401ac18cce7214c316ea9b4?spm=1055.2569.3001.10083&amp;kwd=class) MoveItExample(Node):

# class JacobianSolver(Node):

#     def __init__(self):
#         super().__init__('moveit_jacobian_solver')
        
#         self.move_group = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
#         self.get_logger().info('Waiting for compute_cartesian_path service...')
        
#         while not self.move_group.wait_for_service(timeout_sec=5.0):
#             self.get_logger().info('compute_cartesian_path service not available, waiting again...')
#         self.pose_stamped = PoseStamped()
#         # ... 设置初始或目标位置 ...
