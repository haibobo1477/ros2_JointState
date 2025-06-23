import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
        )
        
        self.declare_parameter('joint_positions', [-1.0, -0.2, 0.5, 0.0, -0.5, 1.57])
        # self.declare_parameter('joint_positions', [0.68239976, -0.62131946, 0.30138218, 1.94006179, -0.74264804, -2.05452146])
        
        
        self.joint_positions = self.get_parameter('joint_positions').value
        
        
        self.timer = self.create_timer(2.0, self.execute_task)
        
        
    def execute_task(self):
        
        # self.bot.arm.go_to_home_pose()
        self.bot.arm.set_joint_positions(self.joint_positions)
        self.bot.arm.go_to_home_pose() 
        # self.bot.arm.go_to_sleep_pose()
        
        self.get_logger().info("Task completed!")
        
        
        

def main(args=None):
    
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()






  


