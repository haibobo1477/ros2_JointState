#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from interbotix_xs_msgs.msg import JointGroupCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import modern_robotics as mr
import math 
from math import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt




class FrameListener(Node):
  """
  Subclass of the Node class.
  The class listens to coordinate transformations and 
  publishes the end-effector velocity at a specific time interval.
  """
  def __init__(self):
    super().__init__('joint_Jacobian_velocity')
    
    
    self.time_list = []
    self.linear_vel_list = []
    self.angular_vel_list = []
    
    self.linear_error_list = []
    self.angular_error_list = []
    
    
    self.declare_parameter('target_frame', 'vx300s/ee_gripper_link')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    
   
    self.publisher_vel = self.create_publisher(Twist, 'ee_twist_from_tf', 1)
    
    self.publisher_vel_err = self.create_publisher(Twist, 'ee_twist_error', 1)
    
    # Create the subscriber
    self.subscription = self.create_subscription(JointState, '/vx300s/joint_states', self.listener_callback, 1)
    
    self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz

    self.homogeneous_matrix_old = np.eye(4)
    self.ti = self.get_clock().now().nanoseconds / 1e9
    
    
    # Create joint position variable (initialization)
    self.angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    self.joint_velocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Define your jacobian matrix which is dependent on joint positions (angles) (make sure to show your calculation in your report)
    # all zero elements of the matrix should be calculated and entered in this matrix as a function of joint angles
    self.Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                           [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                           [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                           [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                           [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                           [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T
    

  def on_timer(self):
    from_frame_rel = self.target_frame
    to_frame_rel = 'vx300s/base_link'

    try:
      trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
      
    except TransformException as ex:
      self.get_logger().warn(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return

    t = trans.transform.translation
    r = trans.transform.rotation
      

      # Get the homogeneous matrix (explain these in your report)
    homogeneous_matrix = self.calcu_Matrix(r.x, r.y, r.z, r.w)
    homogeneous_matrix[0, 3] = t.x
    homogeneous_matrix[1, 3] = t.y
    homogeneous_matrix[2, 3] = t.z

      # Compute the time derivative of T using numerical differentiation
    homogeneous_matrix_deriv = (homogeneous_matrix - self.homogeneous_matrix_old) / 0.1
    self.homogeneous_matrix_old = homogeneous_matrix
    homogeneous_matrix_inv = np.linalg.inv(homogeneous_matrix)

      # Compute the matrix form of the twist: V_hat = dT * T^-1
    vel_brack = homogeneous_matrix_deriv @ homogeneous_matrix_inv

      # Angular velocity is extracted from skew-symmetric part
    w_hat = vel_brack[0:3, 0:3]
    ang_vel = np.array([
        w_hat[2, 1],
        w_hat[0, 2],
        w_hat[1, 0]
      ])

    trans_vel = vel_brack[0:3, 3]

      # Publish the velocity message
    vel_msg = Twist()
    vel_msg.linear.x = trans_vel[0]
    vel_msg.linear.y = trans_vel[1]
    vel_msg.linear.z = trans_vel[2]
    vel_msg.angular.x = ang_vel[0]
    vel_msg.angular.y = ang_vel[1]
    vel_msg.angular.z = ang_vel[2]
    self.publisher_vel.publish(vel_msg)
      
      # self.get_logger().info(
      #   f"EE Twist: Linear → [{trans_vel[0]:.3f}, {trans_vel[1]:.3f}, {trans_vel[2]:.3f}], "
      #   f"Angular → [{ang_vel[0]:.3f}, {ang_vel[1]:.3f}, {ang_vel[2]:.3f}]"
      # )
      
    # current_time = self.get_clock().now().nanoseconds / 1e9 - self.ti
    current_time = self.get_clock().now().nanoseconds / 1e9 - self.ti

    # plot
    # self.time_list.append(current_time)
    # self.linear_vel_list.append(trans_vel.tolist())
    # self.angular_vel_list.append(ang_vel.tolist())
      
      # publish velocity commands
   
    
      # Compute twist using jacobian
    self.J = mr.JacobianSpace(self.Slist, self.angles)
    vel_from_jac = self.J @ self.joint_velocities
    
                
       # Jacobian to twist
    vel_jac_trans = vel_from_jac[0:3].flatten()
    vel_jac_ang = vel_from_jac[3:6].flatten()

      # twist error
    err_linear = trans_vel - vel_jac_trans
    err_angular = ang_vel - vel_jac_ang
      
      # Publish the velocity error message
    vel_err_msg = Twist()
    vel_err_msg.linear.x = err_linear[0]
    vel_err_msg.linear.y = err_linear[1]
    vel_err_msg.linear.z = err_linear[2]
    vel_err_msg.angular.x = err_angular[0]
    vel_err_msg.angular.y = err_angular[1]
    vel_err_msg.angular.z = err_angular[2]
    
    
    self.get_logger().info(
         f"[Twist Error] Linear: x={err_linear[0]:.4f}, y={err_linear[1]:.4f}, z={err_linear[2]:.4f} | "
         f"Angular: x={err_angular[0]:.4f}, y={err_angular[1]:.4f}, z={err_angular[2]:.4f}"
    )
    
    
    

    
    self.time_list.append(current_time)
    self.linear_error_list.append(err_linear.tolist())
    self.angular_error_list.append(err_angular.tolist())
    
    
    self.publisher_vel_err.publish(vel_err_msg)
    
  

  def calcu_Matrix(self, qx, qy, qz, qw):
    """
    Convert quaternion (qx, qy, qz, qw) to 4x4 homogeneous transformation matrix.
    We set translation to zero here and fill in rotation matrix only.
    """
    rotation_matrix = np.array([
      [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
      [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
      [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    return homogeneous_matrix
  
  def listener_callback(self, data: JointState):
        """
        Callback function.
        """
        # Display the message on the console
        self.angles =  np.array([data.position[0],
                                 data.position[1],
                                 data.position[2],
                                 data.position[3],
                                 data.position[4], 
                                 data.position[5]])
        
        self.joint_velocities = np.array([data.velocity[0],
                                          data.velocity[1],
                                          data.velocity[2],
                                          data.velocity[3],
                                          data.velocity[4],
                                          data.velocity[5]])
        
        # self.get_logger().info(
        # f"Joint positions: {np.round(self.angles, 3)} | Velocities: {np.round(self.joint_velocities, 3)}"
        #  )
         
  
  
  def plot_twist_data(self):
    time = self.time_list
    linear_err = np.array(self.linear_error_list)
    angular_err = np.array(self.angular_error_list)
    
    if len(self.time_list) == 0:
        print("No error data to plot.")
        return

    plt.figure(figsize=(12, 6))

    
    plt.subplot(2, 1, 1)
    plt.plot(time, linear_err[:, 0], label='Error Vx')
    plt.plot(time, linear_err[:, 1], label='Error Vy')
    plt.plot(time, linear_err[:, 2], label='Error Vz')
    plt.ylabel("Linear Velocity Error (m/s)")
    plt.title("Twist Error: Linear and Angular")
    plt.legend()
    plt.grid()

    
    plt.subplot(2, 1, 2)
    plt.plot(time, angular_err[:, 0], label='Error Wx')
    plt.plot(time, angular_err[:, 1], label='Error Wy')
    plt.plot(time, angular_err[:, 2], label='Error Wz')
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity Error (rad/s)")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig("twist_error_plot.png")
    plt.show()

    


def main(args=None):
  rclpy.init(args=args)
  frame_listener_node = FrameListener()
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    print("Shutting down... plotting results.")
    frame_listener_node.plot_twist_data()
    frame_listener_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()




# def plot_twist_data(self):
    
  #   time = self.time_list
  #   linear = np.array(self.linear_vel_list)  # shape: [N, 3]
  #   angular = np.array(self.angular_vel_list)

  #   plt.figure(figsize=(12, 6))
    
  #   if len(self.time_list) == 0:
  #     print("No data to plot.")
  #     return
  #   else:
  #     # Linear velocity
  #     plt.subplot(2, 1, 1)
  #     plt.plot(time, linear[:, 0], label='Vx')
  #     plt.plot(time, linear[:, 1], label='Vy')
  #     plt.plot(time, linear[:, 2], label='Vz')
  #     plt.ylabel("Linear Velocity (m/s)")
  #     plt.title("End-effector Linear and Angular Velocity")
  #     plt.legend()
  #     plt.grid()

  # # Angular velocity
  #     plt.subplot(2, 1, 2)
  #     plt.plot(time, angular[:, 0], label='Wx')
  #     plt.plot(time, angular[:, 1], label='Wy')
  #     plt.plot(time, angular[:, 2], label='Wz')
  #     plt.xlabel("Time (s)")
  #     plt.ylabel("Angular Velocity (rad/s)")
  #     plt.legend()
  #     plt.grid()

  #     plt.tight_layout()
  #     plt.savefig("ee_twist_plot.png")
  #     plt.show()