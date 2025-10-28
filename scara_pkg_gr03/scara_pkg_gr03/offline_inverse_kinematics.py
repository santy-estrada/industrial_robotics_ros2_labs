#!/usr/bin/env python3
"""
Offline Inverse Kinematics Node
================================
This node receives ALL waypoints at once and computes ALL joint configurations offline.
Unlike the online version, this processes the entire trajectory upfront.

Topics:
-------
Subscribers:
    - trajectory_waypoints (PointCloud): All waypoints to process

Publishers:
    - joint_configurations (String): JSON array of all joint configs
      Format: [{"j1": deg, "j2": deg, "j3": mm}, ...]

Parameters:
-----------
    - elbow_up (bool): True for elbow up, False for elbow down
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import numpy as np
import json


class OfflineInverseKinematics(Node):
    def __init__(self):
        super().__init__('offline_inverse_kinematics')
        
        # Parameters
        self.declare_parameter('elbow_up', False)
        self.elbow_up = self.get_parameter('elbow_up').value
        
        # Robot parameters
        self.r1 = 0.16  # First arm length (m)
        self.r2 = 0.143  # Second arm length (m)
        self.delta3 = 0.0  # Prismatic joint offset (m)
        
        self.get_logger().info(f"=== Offline Inverse Kinematics Configuration ===")
        self.get_logger().info(f"Elbow configuration: {'UP' if self.elbow_up else 'DOWN'}")
        self.get_logger().info(f"Robot parameters: r1={self.r1}m, r2={self.r2}m")
        
        # QoS for latched messages
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriber
        self.create_subscription(
            PointCloud, 
            'trajectory_waypoints', 
            self.waypoints_callback, 
            qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(String, 'joint_configurations', qos)
        
        self.get_logger().info("✅ Waiting for trajectory waypoints...")
    
    def waypoints_callback(self, msg: PointCloud):
        """Process all waypoints and compute joint configurations."""
        num_waypoints = len(msg.points)
        self.get_logger().info(f"📥 Received {num_waypoints} waypoints. Computing inverse kinematics...")
        
        configurations = []
        valid_count = 0
        
        for idx, point in enumerate(msg.points):
            # Convert from mm to m
            px = point.x / 1000.0
            py = point.y / 1000.0
            pz = point.z / 1000.0
            
            # Compute inverse kinematics
            config = self._compute_ik(px, py, pz)
            
            if config is not None:
                configurations.append(config)
                valid_count += 1
            else:
                # Keep previous configuration if IK fails
                if configurations:
                    self.get_logger().warn(f"⚠️  IK failed for waypoint {idx}, using previous config")
                    configurations.append(configurations[-1])
                else:
                    self.get_logger().error(f"❌ IK failed for first waypoint!")
                    return
            
            # Progress indicator
            if (idx + 1) % 100 == 0:
                self.get_logger().info(f"   Processed {idx + 1}/{num_waypoints} waypoints...")
        
        self.get_logger().info(f"✅ Computed {valid_count}/{num_waypoints} valid configurations")
        
        # Publish as JSON
        msg_out = String()
        msg_out.data = json.dumps(configurations)
        self.publisher.publish(msg_out)
        
        self.get_logger().info(f"📤 Published {len(configurations)} joint configurations")
    
    def _compute_ik(self, px, py, pz):
        """Compute inverse kinematics for a single point."""
        # Joint 1 and 2 (revolute)
        D = round((px**2 + py**2 - self.r1**2 - self.r2**2) / (2 * self.r1 * self.r2), 6)
        
        if abs(D) > 1.0:
            return None
        
        # Choose solution based on elbow_up parameter
        theta2_opt = [math.acos(D), -math.acos(D)]
        chosen_idx = 0 if self.elbow_up else 1
        theta2 = theta2_opt[chosen_idx]
        
        # Check theta2 limits
        if abs(math.degrees(theta2)) > 120:
            return None
        
        # Compute theta1
        mat = np.array([
            [self.r1 + self.r2 * math.cos(theta2), -self.r2 * math.sin(theta2)],
            [self.r2 * math.sin(theta2), self.r1 + self.r2 * math.cos(theta2)]
        ])
        
        if np.linalg.det(mat) == 0:
            return None
        
        mat_inv = np.linalg.inv(mat)
        vec = np.array([px, py])
        sol = mat_inv @ vec
        theta1 = math.atan2(sol[1], sol[0])
        
        # Check theta1 limits
        if abs(math.degrees(theta1)) > 85:
            return None
        
        # Compute prismatic joint
        prismatic = self.delta3 - pz
        
        if abs(prismatic) > 0.14:  # 140mm limit
            return None
        
        # Return configuration in degrees and mm
        return {
            "j1": math.degrees(theta1),
            "j2": math.degrees(theta2),
            "j3": prismatic * 1000.0  # Convert to mm
        }


def main(args=None):
    rclpy.init(args=args)
    node = OfflineInverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
