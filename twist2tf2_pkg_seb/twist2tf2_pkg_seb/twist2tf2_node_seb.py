#!/usr/bin/env python3
"""
Imports for a ROS 2 (rclpy) node that typically:
- Subscribes to geometry_msgs/Twist (velocity commands).
- Broadcasts a TF transform via tf2_ros.TransformBroadcaster.
- Uses NumPy for vectorized trig and state updates.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist

import math
import numpy as np
from numpy import cos, sin

def quaternion_from_euler(ai, aj, ak):
    """
    Convert intrinsic Euler angles (roll=ai, pitch=aj, yaw=ak)
    into a quaternion [x, y, z, w].

    Args:
        ai (float): roll angle in radians
        aj (float): pitch angle in radians
        ak (float): yaw angle in radians

    Returns:
        np.ndarray: quaternion [x, y, z, w] representing the same orientation
    """    
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def rotz(ang):
    """
    Create a 3x3 rotation matrix for a rotation about the Z-axis.
    Args:
        ang (float): rotation angle in radians
    Returns:
        np.ndarray: 3x3 rotation matrix
    """

    R = np.array([[cos(ang),-sin(ang),0],
                  [sin(ang),cos(ang),0],
                  [0,0,1]])

    return R

def map(x, in_min, in_max, out_min, out_max):
    """
    Remap a number from one range into another.
    Args:
    x (float): input value to map
    in_min (float): lower bound of input range
    in_max (float): upper bound of input range
    out_min (float): lower bound of output range
    out_max (float): upper bound of output range
    Returns:
    float: remapped value of x
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Twist2TF2seb(Node):

    def __init__(self):
        # Initialize parent rclpy Node with name "twist2TF2_seb"
        super().__init__('twist2TF2_seb')

        # Create a TF broadcaster to publish TransformStamped messages onto /tf
        # Used to maintain coordinate frame relationships in ROS 2
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to /twist_msg (geometry_msgs/Twist) coming from the Pico
        # Each received message triggers self.twist2tf2_callback
        self.subscription = self.create_subscription(
            Twist,
            '/twist_msg',
            self.twist2tf2_callback,
            1   # Queue size = 1, latest message replaces older
        )
        self.subscription  # Keep reference to avoid "unused variable" cleanup

        # Simulation timestep (Δt) for Euler integration [s]
        self.dt = 0.1

        # Initial pose (position in x, y, z, orientation theta)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0   # heading/yaw angle in radians

        # Initial velocity states (u = forward, v = lateral, r = yaw rate)
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        # First transform: world → odom
        # Provides a global reference frame ("world") to "odom"
        self.t = TransformStamped()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'odom'

        # Second transform: odom → base_link
        # Represents the robot pose within the odometry frame
        self.t2 = TransformStamped()
        self.t2.header.frame_id = 'odom'
        self.t2.child_frame_id = 'base_link'

        # Create a periodic timer that runs broadcaster_callback()
        # This regularly publishes the updated transforms at Δt intervals
        self.timer_broadcaster = self.create_timer(
            self.dt,
            self.broadcaster_callback
        )

    def broadcaster_callback(self):
        """
        Periodic callback that integrates velocity into pose
        and publishes TF transforms:
            world -> odom
            odom  -> base_link
        """
            
        # Compose velocity vector (u: forward, v: lateral, r: yaw rate)
        xi = np.array([[self.u],
                    [self.v],
                    [self.r]])

        # Rotate velocity vector into global/world-aligned frame
        # rotz(self.theta) applies current heading to map body-frame velocities
        eta_dot = rotz(self.theta) @ xi

        # Timestamp each transform with current ROS time
        self.t.header.stamp = self.get_clock().now().to_msg()   # world->odom
        self.t2.header.stamp = self.get_clock().now().to_msg()  # odom->base_link

        # Integrate angular velocity (yaw rate) to update orientation (theta)
        self.theta = self.theta + eta_dot[2, 0] * self.dt

        # ---- ORIENTATIONS ----
        # world->odom orientation (fixed to identity, no rotation)
        # Equivalent to quaternion [0, 0, 0, 1] (identity rotation)
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0

        # odom->base_link orientation
        # Use quaternion from Euler (roll=0, pitch=0, yaw=theta)
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        self.t2.transform.rotation.x = q[0]
        self.t2.transform.rotation.y = q[1]
        self.t2.transform.rotation.z = q[2]
        self.t2.transform.rotation.w = q[3]

        # ---- POSITIONS ----
        # Integrate linear velocity into position
        # x = forward displacement, y = lateral displacement
        self.x = self.x + eta_dot[0, 0] * self.dt
        self.y = self.y + eta_dot[1, 0] * self.dt
        # z remains zero in this planar example

        # world->odom translation (keeps odom origin in world frame)
        self.t.transform.translation.x = float(self.x)
        self.t.transform.translation.y = float(self.y)
        self.t.transform.translation.z = 0.0

        # odom->base_link translation (robot assumed at odom origin)
        self.t2.transform.translation.x = 0.0
        self.t2.transform.translation.y = 0.0
        self.t2.transform.translation.z = 0.0

        # ---- PUBLISH ----
        # Send static/dynamic transforms to TF tree
        self.tf_broadcaster.sendTransform(self.t)   # world -> odom
        self.tf_broadcaster.sendTransform(self.t2)  # odom -> base_link


    def twist2tf2_callback(self, msg):
        """
        Callback for subscribed /twist_msg (geometry_msgs/Twist).
        Updates body-frame velocities (u, v, r) that will be
        integrated later in broadcaster_callback().
        """

        # read the msg and extract the content
        self.u = map(msg.linear.x,0,255,-5,5)
        self.v = msg.linear.y 
        self.r = map(msg.angular.z,0,255,-5,5)

        print("new u: {:.4f}, r: {:.4f}".format(self.u,self.r))
        
def main():
    rclpy.init()
    node = Twist2TF2seb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()