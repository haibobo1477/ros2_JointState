import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def main(args=None):
    rclpy.init(args=args)

    # 初始化 Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )

    # 设置末端执行器的目标位姿 (逆运动学分析 + 执行)
    joint_positions, success = bot.arm.set_ee_pose_components(
        x=0.3,
        y=0.1,
        z=0.5,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,     # vx300s 是 6DOF，可以设置 yaw
        execute=True,
        blocking=True
    )

    if success:
        print(f"IK solved! Joint positions: {joint_positions}")
    else:
        print("IK failed to find a valid solution.")

    # bot.shutdown()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
