import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver_client')

        # 创建服务客户端
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait for /compute_ik serivce available...')
            
            
            
        # 创建 joint_states 订阅者
        self.subscription = self.create_subscription(
            JointState,
            '/vx300s/joint_states',
            self.joint_state_callback,
            10
        )    
        
        
         # self.timer = self.create_timer(0.2, self.send_fk_request)
        self.get_logger().info('Subscribed to /vx300s/joint_states')
        
        
        
    def joint_state_callback(self, msg: JointState):
        # 提取关节名和角度（可根据需要过滤不相关的关节）
        joint_names = [
            'waist', 
            'shoulder', 
            'elbow',
            'forearm_roll', 
            'wrist_angle', 
            'wrist_rotate'
        ]
        joint_position_map = dict(zip(msg.name, msg.position))

        try:
            joint_positions = [joint_position_map[name] for name in joint_names]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint: {e}")
            return

        # 构造服务请求
        request = GetPositionIK.Request()
        
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joint_positions
        
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = 'interbotix_arm'
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = joint_state
        
        request.ik_request.ik_link_name = 'vx300s/ee_gripper_link'
        request.ik_request.timeout = Duration(sec=1)
        # request.ik_request.attempts = 5
        request.ik_request.avoid_collisions = True

        # 设置目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'vx300s/base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0
        
        request.ik_request.pose_stamped = target_pose

        # 发送请求
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_ik_response)

    def handle_ik_response(self, future):
        try:
            response = future.result()
            if response.error_code.val == 1:
                joint_names = response.solution.joint_state.name
                joint_positions = response.solution.joint_state.position
                self.get_logger().info('IK solved successfully:')
                for name, pos in zip(joint_names, joint_positions):
                    self.get_logger().info(f'{name}: {pos:.3f}')
            else:
                self.get_logger().error(f'IK failed, error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'service call exception: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()















