import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS



def main(args=None):
   

    # 初始化 Interbotix Manipulator
    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )


# 示例目标末端位姿矩阵（T_sd）
    T_goal = np.array([
        [1, 0, 0, 0.4],  # X方向0.2m
        [0, 1, 0, 0.2],  # Y方向0
        [0, 0, 1, 0.4],  # Z方向0.3m
        [0, 0, 0, 1]
        ])
    
    
    
    # 调用函数
    theta_list, success = bot.arm.set_ee_pose_matrix(
            T_sd=T_goal,
            execute=True,
            moving_time=2.0,
            accel_time=1.0,
            blocking=True
          )

    if success:
        print("运动成功！")
        print("逆解关节角为：", theta_list)
    else:
        print("未找到有效解！")


if __name__ == '__main__':
    main()
 


