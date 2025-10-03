#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

def q_matrix(a,phi,theta,dx,dy):  
    Q_1 = np.array([[1/a, 1/a * np.tan(phi)]])
    Q_2 = np.array([[np.cos(theta), np.sin(theta)],
                    [-np.sin(theta), np.cos(theta)]])
    Q_3 = np.array([[1,0,dx],
                    [0,1,dy]])
    Q = Q_1 @ Q_2 @ Q_3
    return Q

class Inverse_Kinematics_Diff_Node(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("inverse_kinematics_node") # <--- CHANGE ME

        

        # initial conditions
        self.eta = np.array([[0.0],
                             [0.0],
                             [0.0]])  
        
        # vehicle parameters
        self.a = 0.042/2  # wheel radius
        self.theta1 = 0.0
        self.theta2 = np.radians(180)
        self.dy = 0.12766/2 + 0.019/2  # distance between wheels and center of the robot (xacro)
        self.dx = 0.0
        self.phi = 0.0
                
        # angular velocities of the wheels
        self.w1 = 0.0
        self.w2 = 0.0

        self.u = 0.0
        self.r = 0.0
        
        # publishers and subscribers
        qos = QoSProfile(depth=10)
        
        # subscriber
        self.cmd_vel_sub = self.create_subscription(
            msg_type=Twist,
            topic="cmd_vel",
            callback=self.cmd_vel_callback,
            qos_profile=qos
        )
        
        self.wheel_setpoint = self.create_publisher(
            msg_type=Twist,
            topic="wheel_setpoint",
            qos_profile=qos
        )
        # time variables for integration
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def cmd_vel_callback(self, msg):
        self.u = msg.linear.x
        self.r = msg.angular.z
        self.get_logger().info(f'Comando recibido: u={self.u:.2f} m/s, r={self.r:.2f} rad/s')
        
    def timer_callback(self):
        
        Q_w1 = q_matrix(self.a,self.phi,self.theta1,self.dx,self.dy)
        Q_w2 = q_matrix(self.a,self.phi,self.theta2,self.dx,self.dy)

        Q_w1_filtered = np.array([[Q_w1[0,0], Q_w1[0,2]]])
        Q_w2_filtered = np.array([[Q_w2[0,0], Q_w2[0,2]]])

        Q = np.vstack((Q_w1_filtered,Q_w2_filtered))

        psi = np.array([[self.u],
                        [self.r]])
        
        self.w = Q@psi
        self.w1 = self.w[0,0]
        self.w2 = self.w[1,0]
        
        
        # publish joint state
        self.wheel_setpoint_publisher()

        self.get_logger().info(f"w1: {self.w1:.2f}, w2: {self.w2:.2f}")
        
    def wheel_setpoint_publisher(self):
        w_sp = Twist()
        # position
        w_sp.linear.x = self.w1
        w_sp.linear.y = self.w2
        w_sp.linear.z = 0.0       
        # publish the message
        self.wheel_setpoint.publish(w_sp)

def main():
    rclpy.init()
    node = Inverse_Kinematics_Diff_Node() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()