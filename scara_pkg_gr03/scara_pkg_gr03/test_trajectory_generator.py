#!/usr/bin/env python3
"""
Test Trajectory Generator Node
================================
This node generates test trajectories using mathematical equations instead of DXF files.
Useful for testing the offline trajectory system without needing DXF files.

Supported shapes:
- circle: x = r*cos(theta), y = r*sin(theta)
- square: Four line segments
- star: Five-pointed star
- spiral: r increases with theta
- lissajous: x = A*sin(a*t), y = B*sin(b*t)

Topics:
-------
Publishers:
    - dxf_pointcloud (PointCloud): Generated waypoints
    - dxf_figures_info (String): Figure metadata (always 1 figure)

Parameters:
-----------
    - shape_type (str): "circle", "square", "star", "spiral", "lissajous"
    - num_points (int): Number of points to generate
    - radius (float): Radius for circle/spiral (mm)
    - center_x (float): X center position (mm)
    - center_y (float): Y center position (mm)
    - A, B (float): Amplitudes for lissajous (mm)
    - a, b (float): Frequencies for lissajous
    - spiral_turns (float): Number of turns for spiral
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import json
import math


class TestTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('test_trajectory_generator')
        
        # Parameters
        self.declare_parameter('shape_type', 'circle')
        self.declare_parameter('num_points', 100)
        self.declare_parameter('radius', 50.0)
        self.declare_parameter('center_x', 230.0)
        self.declare_parameter('center_y', -70.0)
        self.declare_parameter('A', 50.0)
        self.declare_parameter('B', 50.0)
        self.declare_parameter('a', 3.0)
        self.declare_parameter('b', 2.0)
        self.declare_parameter('spiral_turns', 3.0)
        
        self.shape_type = self.get_parameter('shape_type').value
        self.num_points = self.get_parameter('num_points').value
        self.radius = self.get_parameter('radius').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.A = self.get_parameter('A').value
        self.B = self.get_parameter('B').value
        self.a = self.get_parameter('a').value
        self.b = self.get_parameter('b').value
        self.spiral_turns = self.get_parameter('spiral_turns').value
        
        self.get_logger().info("=== Test Trajectory Generator Configuration ===")
        self.get_logger().info(f"Shape type: {self.shape_type}")
        self.get_logger().info(f"Number of points: {self.num_points}")
        self.get_logger().info(f"Center: ({self.center_x}, {self.center_y}) mm")
        
        # QoS for latched messages
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', qos)
        self.figures_info_pub = self.create_publisher(String, 'dxf_figures_info', qos)
        
        # Generate and publish trajectory after short delay (only once)
        self.timer = self.create_timer(1.0, self.generate_and_publish)
    
    def generate_and_publish(self):
        """Generate the trajectory and publish it once."""
        self.get_logger().info(f"🔧 Generating {self.shape_type} trajectory...")
        
        # Generate points based on shape type
        if self.shape_type == "circle":
            points = self._generate_circle()
        elif self.shape_type == "square":
            points = self._generate_square()
        elif self.shape_type == "star":
            points = self._generate_star()
        elif self.shape_type == "spiral":
            points = self._generate_spiral()
        elif self.shape_type == "lissajous":
            points = self._generate_lissajous()
        else:
            self.get_logger().error(f"❌ Unknown shape type: {self.shape_type}")
            return
        
        self.get_logger().info(f"✅ Generated {len(points)} points")
        
        # Publish PointCloud
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.frame_id = "base_link"
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in points:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0  # Z will be set by trajectory planner
            pointcloud_msg.points.append(pt)
        
        self.pointcloud_pub.publish(pointcloud_msg)
        self.get_logger().info(f"📤 Published {len(points)} points to /dxf_pointcloud")
        
        # Publish figures info
        figures_info = [{
            "type": self.shape_type.upper(),
            "num_points": len(points)
        }]
        
        figures_msg = String()
        figures_msg.data = json.dumps(figures_info)
        self.figures_info_pub.publish(figures_msg)
        self.get_logger().info(f"📤 Published figures info")
        
        # Cancel timer so we only publish once
        self.timer.cancel()
        self.get_logger().info("✅ Test trajectory generation complete. Node will continue running to keep latched messages available.")
    
    def _generate_circle(self):
        """Generate circle: x = r*cos(theta), y = r*sin(theta)"""
        theta = np.linspace(0, 2*np.pi, self.num_points, endpoint=False)
        x = self.center_x + self.radius * np.cos(theta)
        y = self.center_y + self.radius * np.sin(theta)
        return list(zip(x, y))
    
    def _generate_square(self):
        """Generate square with side length = 2*radius"""
        side = 2 * self.radius
        points_per_side = self.num_points // 4
        
        points = []
        # Bottom edge (left to right)
        x = np.linspace(self.center_x - self.radius, self.center_x + self.radius, points_per_side)
        y = np.full(points_per_side, self.center_y - self.radius)
        points.extend(zip(x, y))
        
        # Right edge (bottom to top)
        x = np.full(points_per_side, self.center_x + self.radius)
        y = np.linspace(self.center_y - self.radius, self.center_y + self.radius, points_per_side)
        points.extend(zip(x, y))
        
        # Top edge (right to left)
        x = np.linspace(self.center_x + self.radius, self.center_x - self.radius, points_per_side)
        y = np.full(points_per_side, self.center_y + self.radius)
        points.extend(zip(x, y))
        
        # Left edge (top to bottom)
        x = np.full(points_per_side, self.center_x - self.radius)
        y = np.linspace(self.center_y + self.radius, self.center_y - self.radius, points_per_side)
        points.extend(zip(x, y))
        
        return points
    
    def _generate_star(self):
        """Generate 5-pointed star"""
        angles = []
        for i in range(10):  # 5 outer + 5 inner points
            angle = i * (2 * np.pi / 10) - np.pi/2  # Start at top
            angles.append(angle)
        
        points = []
        for i, angle in enumerate(angles):
            # Alternate between outer and inner radius
            r = self.radius if i % 2 == 0 else self.radius * 0.4
            x = self.center_x + r * np.cos(angle)
            y = self.center_y + r * np.sin(angle)
            
            # Add interpolated points between vertices
            if i > 0:
                x_prev, y_prev = points[-1]
                n_interp = self.num_points // 20
                x_interp = np.linspace(x_prev, x, n_interp, endpoint=False)
                y_interp = np.linspace(y_prev, y, n_interp, endpoint=False)
                points.extend(zip(x_interp[1:], y_interp[1:]))
            
            points.append((x, y))
        
        return points
    
    def _generate_spiral(self):
        """Generate spiral: r increases linearly with theta"""
        theta = np.linspace(0, 2*np.pi*self.spiral_turns, self.num_points)
        r = np.linspace(0, self.radius, self.num_points)
        x = self.center_x + r * np.cos(theta)
        y = self.center_y + r * np.sin(theta)
        return list(zip(x, y))
    
    def _generate_lissajous(self):
        """Generate Lissajous curve: x = A*sin(a*t), y = B*sin(b*t)"""
        t = np.linspace(0, 2*np.pi, self.num_points)
        x = self.center_x + self.A * np.sin(self.a * t)
        y = self.center_y + self.B * np.sin(self.b * t)
        return list(zip(x, y))


def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
