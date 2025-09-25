#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J

class ODOM_Node_P1(Node):
    def __init__(self):
        super().__init__("odom_node_p1")
        self.eta = np.array([[0.0], [0.0], [0.0]])  # [x, y, phi] position from world frame
        
        self.u = 0.1  # forward velocity
        self.v = 0.0  # lateral velocity (usually 0 for differential drive)
        self.r = 0.1  # angular velocity (yaw rate)
        
        self.xi = np.array([[self.u], [self.v], [self.r]])  # velocity vector from robot frame
        
        # publishers and subscribers
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile=qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.t = TransformStamped()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'odom'
        
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        psi = self.eta[2,0]
        J = jacobin_matrix(psi) # 
        eta_dot = J @ self.xi  # kinematic model
        self.eta += eta_dot * self.dt  # Euler integration

        self.odometry_publisher() # publish odometry message
        self.odom_tf_broadcaster() # publish tf message
        self.get_logger().info(f"x: {self.eta[0,0]:.2f}, y: {self.eta[1,0]:.2f}, phi: {self.eta[2,0]:.2f}")

    def odometry_publisher(self):

        now = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position and orientation
        odom_msg.pose.pose.position.x = self.eta[0,0]
        odom_msg.pose.pose.position.y = self.eta[1,0]
        odom_msg.pose.pose.orientation.z = np.sin(self.eta[2,0]/2)
        odom_msg.pose.pose.orientation.w = np.cos(self.eta[2,0]/2)

        # Velocities
        odom_msg.twist.twist.linear.x = self.u
        odom_msg.twist.twist.linear.y = self.v
        odom_msg.twist.twist.angular.z = self.r

        self.odom_pub.publish(odom_msg)

    def odom_tf_broadcaster(self):
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.transform.translation.x = self.eta[0,0]
        self.t.transform.translation.y = self.eta[1,0]
        self.t.transform.rotation.z = np.sin(self.eta[2,0]/2)
        self.t.transform.rotation.w = np.cos(self.eta[2,0]/2)

        self.tf_broadcaster.sendTransform(self.t)

def main():
    rclpy.init()
    node = ODOM_Node_P1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()