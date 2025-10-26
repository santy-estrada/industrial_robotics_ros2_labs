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


def J(psi):
   return np.array([[np.cos(psi), -np.sin(psi), 0],
              [np.sin(psi), np.cos(psi), 0],
              0, 0, 1])

class NODE(Node):
    def __init__(self):
        super().__init__("mr_odom_node_p2")
        
        self.base_link_id = "base_link"
        self.odom_id = "odom"
        
        self.th1 = 0.0
        self.th2 = 0.0
        self.th3 = 0.0
        self.th4 = 0.0
        
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0
        self.w4 = 0.0
        
        self.w = np.array([self.w1], [self.w2], [self.w3], [self.w4])
        
        self.xi = np.array([0.0], [0.0], [0.0])
        
        self.eta = np.array([0.0], [0.0], [0.0])
        
        self.joint_state_pub = self.create_publisher(msg_type=JointState, topic = "joint_states", qos_profile=10)
        self.odom_pub = self.create_publisher(msg_type=Odometry, topic = "odom", qos_profile=10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.dt = 0.1
        
        self.create_timer(self.dt, self.timer_callback)
        
    def timer_callback(self):
        psi = self.eta[2,0]
        Jac = J(psi)
        
        self.w = np.array([self.w1], [self.w2], [self.w3], [self.w4])

        W = np.array
        
        self.xi = W @ self.w
        
        eta_dot = Jac @ self.xi
        
        self.eta += eta_dot*self.dt
        
        self.pub_odom()
        self.broadcast_tf()
        self.pub_joint_states()
        
    def pub_odom(self):
        now = self.get_clock().now()
        odom_msg = Odometry() #

        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_id
        odom_msg.child_frame_id = self.base_link_id

        odom_msg.pose.pose.position.x = self.eta[0,0]
        odom_msg.pose.pose.position.y = self.eta[1,0]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(self.eta[2,0]/2)
        odom_msg.pose.pose.orientation.w = np.cos(self.eta[2,0]/2)

        odom_msg.twist.twist.linear.x = self.xi[0,0]
        odom_msg.twist.twist.linear.y = self.xi[1,0]
        odom_msg.twist.twist.angular.z = self.xi[2,0]

        self.odom_pub.publish(odom_msg)
    
    def broadcast_tf(self):
        self.tf_broadcaster = TransformBroadcaster(self)

        t = TransformStamped()
        t.header.frame_id = self.odom_id
        t.child_frame_id = self.base_link_id

        now = self.get_clock().now()
        t.header.stamp = now.to_msg()
                
        t.transform.translation.x = self.eta[0,0]
        t.transform.translation.y = self.eta[1,0]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(self.eta[2,0]/2)
        t.transform.rotation.w = np.cos(self.eta[2,0]/2)

        self.tf_broadcaster.sendTransform(t)
    
    def pub_joint_states(self):
        self.th1 += self.w1*self.dt
        self.th2 += self.w2*self.dt
        self.th3 += self.w3*self.dt
        self.th4 += self.w4*self.dt


        
        joint_state_msg = JointState()
        joint_state_msg.header.frame_id = self.base_link_id
        joint_state_msg.name = ["chassis_w1", "chassis_2", "chassis_3", "chassis_w4"]

        now = self.get_clock().now()
        joint_state_msg.header.stamp = now.to_msg()


        joint_state_msg.position = [self.th1, self.th2, self.th3, self.th4]
        joint_state_msg.velocity = [self.w1, self.w2, self.w3, self.w4]
        joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self.joint_state_pub.publish(joint_state_msg)

        


        
    
        

def main():
    rclpy.init()
    node = NODE()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()