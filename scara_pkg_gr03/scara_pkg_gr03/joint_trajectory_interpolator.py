#!/usr/bin/env python3
"""
Joint Trajectory Interpolator Node
===================================
This node receives joint configurations and interpolates between them
for joints 1 and 2 only. Joint 3 is passed through without interpolation.

Topics:
-------
Subscribers:
    - joint_configurations (String): JSON array of configurations

Publishers:
    - interpolated_trajectory (String): JSON array of interpolated configurations

Parameters:
-----------
    - interpolation_type (str): "linear", "cubic", or "quintic"
    - min_angle_deg (float): Minimum angle change to keep a waypoint (degrees)
    - points_per_segment (int): Number of interpolation points between configurations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import json
import math


class JointTrajectoryInterpolator(Node):
    def __init__(self):
        super().__init__('joint_trajectory_interpolator')
        
        # Parameters
        self.declare_parameter('interpolation_type', 'quintic')
        self.declare_parameter('min_angle_deg', 0.1)
        self.declare_parameter('points_per_segment', 50)
        
        self.interpolation_type = self.get_parameter('interpolation_type').value
        self.min_angle_deg = self.get_parameter('min_angle_deg').value
        self.points_per_segment = self.get_parameter('points_per_segment').value
        
        self.get_logger().info(f"=== Joint Trajectory Interpolator Configuration ===")
        self.get_logger().info(f"Interpolation type: {self.interpolation_type}")
        self.get_logger().info(f"Min angle filter: {self.min_angle_deg} degrees")
        self.get_logger().info(f"Points per segment: {self.points_per_segment}")
        
        # QoS for latched messages
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriber
        self.create_subscription(
            String,
            'joint_configurations',
            self.configurations_callback,
            qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(String, 'interpolated_trajectory', qos)
        
        self.get_logger().info("✅ Waiting for joint configurations...")
    
    def configurations_callback(self, msg: String):
        """Process configurations and interpolate."""
        try:
            configurations = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse configurations: {e}")
            return
        
        self.get_logger().info(f"📥 Received {len(configurations)} configurations")
        
        # Apply angle filter
        filtered_configs = self._filter_by_angle(configurations)
        self.get_logger().info(f"🔍 After angle filtering: {len(filtered_configs)} configurations")
        
        # Interpolate
        interpolated = self._interpolate_trajectory(filtered_configs)
        self.get_logger().info(f"✅ Generated {len(interpolated)} interpolated configurations")
        
        # Publish
        msg_out = String()
        msg_out.data = json.dumps(interpolated)
        self.publisher.publish(msg_out)
        
        self.get_logger().info(f"📤 Published interpolated trajectory")
    
    def _filter_by_angle(self, configurations):
        """Filter configurations based on minimum angle change."""
        if not configurations:
            return []
        
        filtered = [configurations[0]]  # Always keep first
        
        for config in configurations[1:]:
            last_config = filtered[-1]
            
            # Calculate angle differences for J1 and J2
            dj1 = abs(config['j1'] - last_config['j1'])
            dj2 = abs(config['j2'] - last_config['j2'])
            
            # Keep if either joint moved significantly
            if dj1 >= self.min_angle_deg or dj2 >= self.min_angle_deg:
                filtered.append(config)
        
        # Always keep last configuration
        if len(configurations) > 1 and filtered[-1] != configurations[-1]:
            filtered.append(configurations[-1])
        
        return filtered
    
    def _interpolate_trajectory(self, configurations):
        """Interpolate between filtered configurations."""
        if len(configurations) < 2:
            return configurations
        
        interpolated = []
        
        for i in range(len(configurations) - 1):
            c0 = configurations[i]
            cf = configurations[i + 1]
            
            # Interpolate J1 and J2
            j1_vals = self._interpolate_1d(
                c0['j1'], cf['j1'], 
                self.points_per_segment
            )
            j2_vals = self._interpolate_1d(
                c0['j2'], cf['j2'],
                self.points_per_segment
            )
            
            # J3: Hold the value from c0 (step function, no interpolation)
            # All points in this segment keep the starting value
            j3_val = c0['j3']
            
            # Create interpolated configurations
            for j1, j2 in zip(j1_vals, j2_vals):
                interpolated.append({
                    "j1": float(j1),
                    "j2": float(j2),
                    "j3": float(j3_val)  # Keep constant value from start of segment
                })
        
        # Add final configuration
        interpolated.append(configurations[-1])
        
        return interpolated
    
    def _interpolate_1d(self, q0, qf, n_points):
        """1D interpolation for a single joint."""
        t = np.linspace(0, 1, n_points)
        T = 1.0  # Normalized time
        
        if self.interpolation_type == "linear":
            return q0 + (qf - q0) * t
        
        elif self.interpolation_type == "cubic":
            # Cubic with zero initial and final velocities
            v0, vf = 0.0, 0.0
            a0 = q0
            a1 = v0
            a2 = (3 * (qf - q0) / T**2) - (2 * v0 + vf) / T
            a3 = (2 * (q0 - qf) / T**3) + (v0 + vf) / T**2
            return a0 + a1 * t + a2 * t**2 + a3 * t**3
        
        elif self.interpolation_type == "quintic":
            # Quintic with zero velocities and accelerations
            v0, vf, a0_acc, af_acc = 0.0, 0.0, 0.0, 0.0
            a0 = q0
            a1 = v0
            a2 = a0_acc / 2
            a3 = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0_acc - af_acc) * T**2) / (2 * T**3)
            a4 = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0_acc - 2 * af_acc) * T**2) / (2 * T**4)
            a5 = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0_acc - af_acc) * T**2) / (2 * T**5)
            return a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
        
        else:
            return q0 + (qf - q0) * t


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
