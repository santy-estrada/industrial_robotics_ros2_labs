#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry

def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J

class ODOM_Node_P2(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("odom_node_p2") # <--- CHANGE ME

        # initial conditions
        self.eta = np.array([[0.0],
                             [0.0],
                             [0.0]])  
        
        # vehicle parameters
        self.a = 0.05  # wheel radius
        self.d = 0.2+0.04/2  # distance between wheels and center of the robot
        self.l = 0.2  # distance between front and rear axis
                
        # joint states wheel angle
        self.th1 = 0.0
        self.th2 = 0.0
        self.th3 = 0.0
        self.th4 = 0.0

        # angular velocities of the wheels
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0
        self.w4 = 0.0

        # publishers and subscribers
        qos = QoSProfile(depth=10)
        
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
        
    def timer_callback(self):
        
        psi = self.eta[2,0]
        J = jacobin_matrix(psi)
        
        self.w = np.array([[self.w1],
                           [self.w2],
                           [self.w3],
                           [self.w4]])
        
        # wheel kimematic model
        W = (self.a/4)*np.array([[1, 1, 1, 1],
                                 [1,-1, 1, -1],
                                 [-1/(self.d-self.l), -1/(self.d-self.l), 1/(self.d-self.l), 1/(self.d-self.l)]])
        
        # body velocity vector
        self.xi = W @ self.w
        
        # kinematic model
        eta_dot = J @ self.xi
        
        # euler integration
        self.eta += eta_dot * self.dt
        
        # publish odometry message
        self.odometry_publisher()
        
        # publish tf message
        self.odom_tf_broadcaster()
        
        # publish joint state
        self.joint_state_publisher()

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
        self.th1 += self.w1 * self.dt
        self.th2 += self.w2 * self.dt
        self.th3 += self.w3 * self.dt
        self.th4 += self.w4 * self.dt
        
        # header
        now = self.get_clock().now()
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now.to_msg()
        joint_state_msg.header.frame_id = "base_link"
        joint_state_msg.name = ["front_left_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint", "front_right_wheel_joint"]

        # position
        joint_state_msg.position = [self.th1, self.th2, self.th3, self.th4]

        # velocity
        joint_state_msg.velocity = [self.w1, self.w2, self.w3, self.w4]
        joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0]

        # publish the message
        self.joint_state_pub.publish(joint_state_msg)

def main():
    rclpy.init()
    node = ODOM_Node_P2() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()