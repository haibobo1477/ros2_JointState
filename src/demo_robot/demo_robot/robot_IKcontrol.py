from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import argparse


def main(args=None):
    
    parser = argparse.ArgumentParser(description='Set end-effector pose via terminal')
    parser.add_argument('--x', type=float, default=0.3, help='X position in meters')
    parser.add_argument('--y', type=float, default=0.1, help='Y position in meters')
    parser.add_argument('--z', type=float, default=0.5, help='Z position in meters')
    parser.add_argument('--roll', type=float, default=0.0, help='Roll in radians')
    parser.add_argument('--pitch', type=float, default=0.0, help='Pitch in radians')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw in radians')
    args = parser.parse_args()


    # initialize Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        moving_time=4.0,      
        accel_time=0.1        
    )

    # inverse
    joint_positions, success = bot.arm.set_ee_pose_components(
        x=args.x,
        y=args.y,
        z=args.z,
        roll=args.roll,
        pitch=args.pitch,
        yaw=args.yaw,
        execute=True,
        blocking=True,
    )

    if success:
        print(f"IK solved! Joint positions: {joint_positions}")
    else:
        print("IK failed to find a valid solution.")


if __name__ == '__main__':
    main()
