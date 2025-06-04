import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class FKSolver(Node):
    def __init__(self):
        super().__init__('fk_solver_client')
        self.cli = self.create_client(GetPositionFK, '/compute_fk')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')
            # 设置循环定时器，每 1 秒调用一次
        
           
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
        request = GetPositionFK.Request()
        request.header.frame_id = 'vx300s/base_link'
        request.fk_link_names = ['vx300s/ee_gripper_link']

        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joint_positions

        robot_state = RobotState()
        robot_state.joint_state = joint_state
        request.robot_state = robot_state

        # 异步调用 FK 服务
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_fk_response)


    def handle_fk_response(self, future):
        try:
            response = future.result()
            if response and response.pose_stamped:
                pose = response.pose_stamped[0].pose
                self.get_logger().info(
                    f"EE Pose -> x: {pose.position.x:.3f}, y: {pose.position.y:.3f}, z: {pose.position.z:.3f}"
                )
            else:
                self.get_logger().warn('FK service returned no pose.')
        except Exception as e:
            self.get_logger().error(f'FK service call failed: {e}')
            
def main():
    rclpy.init()
    node = FKSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

 
 
 
 


