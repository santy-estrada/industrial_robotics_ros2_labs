#!/usr/bin/env python3
"""
Offline Trajectory Generator Node
==================================
This node generates interpolated waypoints from DXF data (similar to scara_trajectory_planner_v2)
but publishes ALL waypoints at once as a PointCloud for offline processing.

Key differences from scara_trajectory_planner_v2:
- num_points = time * density_param (NOT time * publish_rate)
- Publishes all waypoints at once in a PointCloud (NOT one by one in Twist)
- No real-time publishing loop

Topics:
-------
Subscribers:
    - dxf_pointcloud (PointCloud): Raw waypoints from DXF exporter
    - dxf_figures_info (String): Figure metadata

Publishers:
    - trajectory_waypoints (PointCloud): All interpolated waypoints at once

Parameters:
-----------
    - interpolation_type (str): "linear", "cubic", or "quintic"
    - safe_height (float): Z height for transitions (mm)
    - prismatic_down_height (float): Z height during work (mm)
    - time_work_segment (float): Time for work segments (s)
    - time_segment_close (float): Time for closing segments (s)
    - time_prismatic_up (float): Time for raising (s)
    - time_prismatic_down (float): Time for lowering (s)
    - time_xy_transition (float): Time for XY transitions (s)
    - waypoint_density (float): Waypoints per second for interpolation
    - home_x, home_y (float): Home position (mm)
    - initial_position_repeats (int): Repeats of first position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import json


class OfflineTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('offline_trajectory_generator')
        
        # Parameters (same as scara_trajectory_planner_v2 but with density instead of publish_rate)
        self.declare_parameter('interpolation_type', 'quintic')
        self.declare_parameter('safe_height', 20.0)
        self.declare_parameter('prismatic_down_height', 10.0)
        self.declare_parameter('time_work_segment', 20.0)
        self.declare_parameter('time_segment_close', 20.0)
        self.declare_parameter('time_prismatic_up', 0.5)
        self.declare_parameter('time_prismatic_down', 0.5)
        self.declare_parameter('time_xy_transition', 20.0)
        self.declare_parameter('waypoint_density', 20.0)  # waypoints per second
        self.declare_parameter('home_x', 303.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('initial_position_repeats', 10)
        
        self.interpolation_type = self.get_parameter('interpolation_type').value
        self.safe_height = self.get_parameter('safe_height').value
        self.prismatic_down_height = self.get_parameter('prismatic_down_height').value
        self.time_work_segment = self.get_parameter('time_work_segment').value
        self.time_segment_close = self.get_parameter('time_segment_close').value
        self.time_prismatic_up = self.get_parameter('time_prismatic_up').value
        self.time_prismatic_down = self.get_parameter('time_prismatic_down').value
        self.time_xy_transition = self.get_parameter('time_xy_transition').value
        self.waypoint_density = self.get_parameter('waypoint_density').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.initial_position_repeats = self.get_parameter('initial_position_repeats').value
        
        self.get_logger().info("=== Offline Trajectory Generator Configuration ===")
        self.get_logger().info(f"Interpolation type: {self.interpolation_type}")
        self.get_logger().info(f"Waypoint density: {self.waypoint_density} points/second")
        self.get_logger().info(f"Safe height: {self.safe_height} mm")
        self.get_logger().info(f"Work height: {self.prismatic_down_height} mm")
        
        # QoS for latched messages
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscribers
        self.create_subscription(PointCloud, 'dxf_pointcloud', self.pointcloud_callback, qos)
        self.create_subscription(String, 'dxf_figures_info', self.figures_info_callback, qos)
        
        # Publishers
        self.publisher = self.create_publisher(PointCloud, 'trajectory_waypoints', qos)
        self.rviz_publisher = self.create_publisher(Path, 'planned_trajectory_rviz', 10)
        
        # State
        self.all_points = []
        self.figures_info = []
        
        self.get_logger().info("✅ Waiting for DXF data...")
    
    def pointcloud_callback(self, msg: PointCloud):
        """Receive raw waypoints from DXF."""
        self.all_points = np.array([[pt.x, pt.y, pt.z] for pt in msg.points])
        self.get_logger().info(f"📥 Received {len(self.all_points)} raw points")
        
        if self.figures_info:
            self.generate_trajectory()
    
    def figures_info_callback(self, msg: String):
        """Receive figure metadata."""
        try:
            self.figures_info = json.loads(msg.data)
            self.get_logger().info(f"📥 Received figures info: {self.figures_info}")
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing JSON: {e}")
            return
        
        if len(self.all_points) > 0:
            self.generate_trajectory()
    
    def generate_trajectory(self):
        """Generate complete interpolated trajectory (same logic as scara_trajectory_planner_v2)."""
        if not self.all_points.size or not self.figures_info:
            return
        
        self.get_logger().info("🔧 Generating interpolated trajectory...")
        full_trajectory = []
        current_point_idx = 0
        
        first_point = self.all_points[0]
        
        # Repeat starting point at safe height (gives robot time to reach position)
        for _ in range(self.initial_position_repeats):
            full_trajectory.append((first_point[0], first_point[1], self.safe_height))
        
        for fig_idx, fig in enumerate(self.figures_info):
            num_points = fig["num_points"]
            fig_type = fig["type"]
            
            figure_points = self.all_points[current_point_idx:current_point_idx + num_points]
            current_point_idx += num_points
            
            # Close figure if needed
            if len(figure_points) > 1:
                first_point = figure_points[0]
                last_point = figure_points[-1]
                distance_to_close = np.sqrt((last_point[0] - first_point[0])**2 + 
                                           (last_point[1] - first_point[1])**2)
                
                if distance_to_close > 0.1:
                    figure_points = np.vstack([figure_points, figure_points[0]])
            
            # Determine time per segment
            if "CIRCLE" in fig_type or "ARC" in fig_type:
                time_per_segment = self.time_work_segment / 4
            else:
                time_per_segment = self.time_work_segment
            
            # First figure: move down
            if fig_idx == 0:
                first_point = figure_points[0]
                down_traj = self._interpolate_segment(
                    first_point[0], first_point[1], self.safe_height,
                    first_point[0], first_point[1], self.prismatic_down_height,
                    self.time_prismatic_down
                )
                full_trajectory.extend(down_traj)
            
            # Interpolate all segments
            for i in range(len(figure_points) - 1):
                p1 = figure_points[i]
                p2 = figure_points[i + 1]
                
                is_closing = (i == len(figure_points) - 2)
                segment_time = self.time_segment_close if is_closing else time_per_segment
                
                segment_traj = self._interpolate_segment(
                    p1[0], p1[1], self.prismatic_down_height,
                    p2[0], p2[1], self.prismatic_down_height,
                    segment_time
                )
                full_trajectory.extend(segment_traj)
            
            # Transition to next figure
            if fig_idx < len(self.figures_info) - 1:
                last_point = figure_points[-1]
                next_first_point = self.all_points[current_point_idx]
                
                # Up
                up_traj = self._interpolate_segment(
                    last_point[0], last_point[1], self.prismatic_down_height,
                    last_point[0], last_point[1], self.safe_height,
                    self.time_prismatic_up
                )
                full_trajectory.extend(up_traj)
                
                # XY transition
                xy_traj = self._interpolate_segment(
                    last_point[0], last_point[1], self.safe_height,
                    next_first_point[0], next_first_point[1], self.safe_height,
                    self.time_xy_transition
                )
                full_trajectory.extend(xy_traj)
                
                # Down
                down_traj = self._interpolate_segment(
                    next_first_point[0], next_first_point[1], self.safe_height,
                    next_first_point[0], next_first_point[1], self.prismatic_down_height,
                    self.time_prismatic_down
                )
                full_trajectory.extend(down_traj)
        
        # Final up
        last_point = full_trajectory[-1]
        final_up_traj = self._interpolate_segment(
            last_point[0], last_point[1], last_point[2],
            last_point[0], last_point[1], self.safe_height,
            self.time_prismatic_up
        )
        full_trajectory.extend(final_up_traj)
        
        # Jump to home position (no interpolation)
        for _ in range(20):
            full_trajectory.append((self.home_x, self.home_y, self.safe_height))
        
        self.get_logger().info(f"✅ Generated {len(full_trajectory)} interpolated waypoints")
        
        # Publish RViz visualization
        self.publish_rviz_trajectory(full_trajectory)
        
        # Publish all at once as PointCloud
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.frame_id = "base_link"
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, z in full_trajectory:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            pointcloud_msg.points.append(pt)
        
        self.publisher.publish(pointcloud_msg)
        self.get_logger().info(f"📤 Published {len(pointcloud_msg.points)} waypoints for IK processing")
    
    def _interpolate_segment(self, x0, y0, z0, xf, yf, zf, time_duration):
        """Interpolate between two points. num_points = time * density."""
        num_points = max(2, int(time_duration * self.waypoint_density))
        
        x_vals = self._interpolate_1d(x0, xf, time_duration, num_points)
        y_vals = self._interpolate_1d(y0, yf, time_duration, num_points)
        z_vals = self._interpolate_1d(z0, zf, time_duration, num_points)
        
        return [(x, y, z) for x, y, z in zip(x_vals, y_vals, z_vals)]
    
    def _interpolate_1d(self, q0, qf, T, n_points):
        """1D interpolation."""
        t = np.linspace(0, T, n_points)
        
        if self.interpolation_type == "linear":
            return q0 + (qf - q0) * (t / T)
        
        elif self.interpolation_type == "cubic":
            v0, vf = 0.0, 0.0
            a0 = q0
            a1 = v0
            a2 = (3 * (qf - q0) / T**2) - (2 * v0 + vf) / T
            a3 = (2 * (q0 - qf) / T**3) + (v0 + vf) / T**2
            return a0 + a1 * t + a2 * t**2 + a3 * t**3
        
        elif self.interpolation_type == "quintic":
            v0, vf, a0_acc, af_acc = 0.0, 0.0, 0.0, 0.0
            a0 = q0
            a1 = v0
            a2 = a0_acc / 2
            a3 = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0_acc - af_acc) * T**2) / (2 * T**3)
            a4 = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0_acc - 2 * af_acc) * T**2) / (2 * T**4)
            a5 = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0_acc - af_acc) * T**2) / (2 * T**5)
            return a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
        
        else:
            return q0 + (qf - q0) * (t / T)
    
    def publish_rviz_trajectory(self, full_trajectory):
        """Publish the complete trajectory to RViz for visualization (in meters)."""
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in full_trajectory:
            x, y, z = point
            
            pose = PoseStamped()
            pose.header = path_msg.header
            # Convert from mm to meters for RViz
            pose.pose.position.x = x / 1000.0
            pose.pose.position.y = -y / 1000.0
            pose.pose.position.z = z / 1000.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)

        self.rviz_publisher.publish(path_msg)
        self.get_logger().info(f"📐 Published trajectory to RViz: {len(path_msg.poses)} poses (in meters)")


def main(args=None):
    rclpy.init(args=args)
    node = OfflineTrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
