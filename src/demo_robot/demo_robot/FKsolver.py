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
    
    
    
    bot.arm.set_joint_positions([0.544863915451544, 0.0894382850393974, -0.0220814542667834, -1.68139787529117, 0.548594008163655, 1.90022246326824])

if __name__ == '__main__':
    main()






  


