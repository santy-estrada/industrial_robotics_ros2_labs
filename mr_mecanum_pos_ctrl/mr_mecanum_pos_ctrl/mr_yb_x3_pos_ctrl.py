#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry

def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J

class mr_yb_x3_pos_ctrl(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("mr_yb_x3_pos_ctrl") # <--- CHANGE ME

        # initial conditions
        self.eta = np.array([[0.0],
                             [0.0],
                             [0.0]])  
        
        # vehicle parameters
        self.a = 6.5/100         # 0.05  # wheel radius
        self.d = (17.5/2)/100    # 0.2+0.04/2  # distance between wheels and center of the robot
        self.l = 15.5/100        # 0.2  # distance between front and rear axis
                
        # joint states wheel angle
        self.th1 = 0.0
        self.th2 = 0.0
        self.th3 = 0.0
        self.th4 = 0.0

        # position control variables
        self.x_des = -1.0
        self.y_des = 1.0
        self.phi_des = 0.0

        # velocity control variables
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.phi_vel = 0.0
        
        # body velocities
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        # publishers and subscribers
        qos = QoSProfile(depth=10)

        
        # subscriber
        self.vel_raw_sub = self.create_subscription(
            msg_type=Twist,
            topic="vel_raw",
            callback=self.vel_raw_callback,
            qos_profile=qos
        )
        
        # publisher
        self.cmd_vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel_nav",
            qos_profile=qos
        )
        
        # odom_publisher
        self.odom_pub = self.create_publisher(
            msg_type=Odometry, # <--- CHANGE ME
            topic="odom",
            qos_profile=qos
        )
        
        self.joint_state_pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=qos
        )

        # tf broadcaster
        # Create a TF broadcaster to publish TransformStamped messages onto /tf
        # Used to maintain coordinate frame relationships in ROS 2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.t = TransformStamped()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'

        # time variables for integration
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def vel_raw_callback(self, msg):        
        self.u = msg.linear.x
        self.v = msg.linear.y
        self.r = msg.angular.z
        
    def timer_callback(self):

        # desired position
        eta_d = np.array([[self.x_des],
                          [self.y_des],
                          [self.phi_des]])
        
        # desired velocity
        etad_d = np.array([[self.x_vel],
                          [self.y_vel],
                          [self.phi_vel]]) 
        
        # position controller gain
        Kp = np.diag([0.25, 0.25, 0.25])
        
        # position error
        e_eta = eta_d - self.eta
        
        # commanded velocity
        eta_cmd = etad_d + Kp @ e_eta
        
        # compute the body velocity vector from the commanded velocity
        phi = self.eta[2,0]
        J = jacobin_matrix(phi)

        xi_cmd = np.linalg.inv(J) @ eta_cmd
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = xi_cmd[0,0]
        cmd_vel_msg.linear.y = xi_cmd[1,0]
        cmd_vel_msg.angular.z = xi_cmd[2,0]
        
        self.get_logger().info(f"u: {xi_cmd[0,0]:.2f}, v: {xi_cmd[1,0]:.2f}, r: {xi_cmd[2,0]:.2f}")
        
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        
        #Q = (1/self.a)*np.array([[1, 1, -(self.d-self.l)],
        #                         [1,-1, -(self.d-self.l)],
        #                         [1, 1,  (self.d-self.l)],
        #                         [1,-1,  (self.d-self.l)]])

        #self.w = Q @ xi_cmd
        
        # wheel motor control
        # Compute the wheel angular velocities from the wheel speeds
        # Here you can implement a motor control algorithm to achieve the desired wheel speeds
        # For simplicity, we assume the wheels can achieve the desired speeds instantaneously
    
        
        # wheel kimematic model
        #W = (self.a/4)*np.array([[1, 1, 1, 1],
        #                         [1,-1, 1, -1],
        #                         [-1/(self.d-self.l), -1/(self.d-self.l), 1/(self.d-self.l), 1/(self.d-self.l)]])
        #
        # body velocity vector
        #self.xi = W @ self.w

        self.xi = np.array([[self.u],
                            [self.v],
                            [self.r]])
        # kinematic model
        eta_dot = J @ self.xi
        
        # euler integration
        self.eta += eta_dot * self.dt
        
        # publish odometry message
        self.odometry_publisher()
        
        # publish tf message
        self.odom_tf_broadcaster()
        
        # publish joint state
        #self.joint_state_publisher()

        self.get_logger().info(f"x: {self.eta[0,0]:.2f}, y: {self.eta[1,0]:.2f}, phi: {self.eta[2,0]:.2f}")
        
    def odometry_publisher(self):
    
        now = self.get_clock().now()
        odom_msg = Odometry() #
        
        # header
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # position and orientation
        odom_msg.pose.pose.position.x = self.eta[0,0]
        odom_msg.pose.pose.position.y = self.eta[1,0]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(self.eta[2,0]/2)
        odom_msg.pose.pose.orientation.w = np.cos(self.eta[2,0]/2)
        
        # linear and angular velocities on {body frame}
        odom_msg.twist.twist.linear.x = self.xi[0,0]
        odom_msg.twist.twist.linear.y = self.xi[1,0]
        odom_msg.twist.twist.angular.z = self.xi[2,0]

        # publish the message
        self.odom_pub.publish(odom_msg)
        
    def odom_tf_broadcaster(self):
        
        # header
        now = self.get_clock().now()
        self.t.header.stamp = now.to_msg()
               
        # position and orientation
        self.t.transform.translation.x = self.eta[0,0]
        self.t.transform.translation.y = self.eta[1,0]
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = np.sin(self.eta[2,0]/2)
        self.t.transform.rotation.w = np.cos(self.eta[2,0]/2)
        
        # send the transformation
        self.tf_broadcaster.sendTransform(self.t)

    def joint_state_publisher(self):
        
        # position integration
        self.th1 += self.w[0,0] * self.dt
        self.th2 += self.w[1,0] * self.dt
        self.th3 += self.w[2,0] * self.dt
        self.th4 += self.w[3,0] * self.dt

        # header
        now = self.get_clock().now()
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now.to_msg()
        joint_state_msg.header.frame_id = "base_link"
        joint_state_msg.name = ["front_left_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint", "front_right_wheel_joint"]

        # position
        joint_state_msg.position = [self.th1, self.th2, self.th3, self.th4]

        # velocity
        joint_state_msg.velocity = [self.w[0,0], self.w[1,0], self.w[2,0], self.w[3,0]]
        joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0]

        # publish the message
        self.joint_state_pub.publish(joint_state_msg)

def main():
    rclpy.init()
    node = mr_yb_x3_pos_ctrl() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()