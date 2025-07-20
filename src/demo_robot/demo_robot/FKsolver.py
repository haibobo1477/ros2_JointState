import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

   

def main(args=None):
        # initialize Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )
    
    
    
    bot.arm.set_joint_positions([1.15447469039224, 0.234123256025709, -0.606685316728335, -1.21433220565213, 1.04794387197896, 1.27006026653244])

if __name__ == '__main__':
    main()






  


