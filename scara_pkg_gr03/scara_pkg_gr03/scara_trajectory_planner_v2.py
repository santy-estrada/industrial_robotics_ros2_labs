#!/usr/bin/env python3
"""
SCARA Trajectory Planner V2
===========================
This node receives waypoints from a DXF parser and generates a smooth trajectory
with the following features:
- 20Hz publishing rate for consistent timing
- 1mm minimum waypoint change filter to reduce redundant publications
- Configurable prismatic joint heights for transitions and path execution
- RViz visualization support (publishes in meters)
- Quintic/Cubic/Linear interpolation support

Topics:
-------
Subscribers:
    - dxf_pointcloud (PointCloud): Waypoints from DXF file
    - dxf_figures_info (String): Figure metadata (type, number of points)

Publishers:
    - desired_pos (Twist): Target position commands (mm, linear.x/y/z)
    - planned_trajectory_rviz (Path): Trajectory for RViz visualization (meters)

Parameters:
-----------
    - interpolation_type (str): "linear", "cubic", or "quintic"
    - safe_height (float): Z height for transitions between figures (mm)
    - prismatic_down_height (float): Z height during path execution (mm)
    - time_work_segment (float): Time for interpolating work segments (s)
    - time_segment_close (float): Time for closing a figure (s)
    - time_prismatic_up (float): Time for raising prismatic joint (s)
    - time_prismatic_down (float): Time for lowering prismatic joint (s)
    - time_xy_transition (float): Time for XY transition between figures (s)
    - publish_rate (float): Rate for publishing waypoints (Hz), default 20Hz
    - min_distance_mm (float): Minimum distance change to publish (mm), default 1.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import json
from std_msgs.msg import String
import math


class ScaraTrajectoryPlannerV2(Node):
    def __init__(self):
        super().__init__('scara_trajectory_planner_v2')

        # ==================== Parameters ====================
        self.declare_parameter('interpolation_type', 'quintic')
        self.declare_parameter('safe_height', 20.0)
        self.declare_parameter('prismatic_down_height', 10.0)
        self.declare_parameter('time_work_segment', 20.0)
        self.declare_parameter('time_segment_close', 20.0)
        self.declare_parameter('time_prismatic_up', 0.5)
        self.declare_parameter('time_prismatic_down', 0.5)
        self.declare_parameter('time_xy_transition', 20.0)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('min_distance_mm', 1.0)  # Minimum distance in mm
        self.declare_parameter('home_x', 303.0)  # Home X position (mm)
        self.declare_parameter('home_y', 0.0)  # Home Y position (mm)
        self.declare_parameter('initial_position_repeats', 10)  # Number of times to send first position

        self.interpolation_type = self.get_parameter('interpolation_type').value
        self.safe_height = self.get_parameter('safe_height').value
        self.prismatic_down_height = self.get_parameter('prismatic_down_height').value
        self.time_work_segment = self.get_parameter('time_work_segment').value
        self.time_segment_close = self.get_parameter('time_segment_close').value
        self.time_prismatic_up = self.get_parameter('time_prismatic_up').value
        self.time_prismatic_down = self.get_parameter('time_prismatic_down').value
        self.time_xy_transition = self.get_parameter('time_xy_transition').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_distance_mm = self.get_parameter('min_distance_mm').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.initial_position_repeats = self.get_parameter('initial_position_repeats').value

        self.get_logger().info(f"=== SCARA Trajectory Planner V2 Configuration ===")
        self.get_logger().info(f"Interpolation type: {self.interpolation_type}")
        self.get_logger().info(f"Safe height (transitions): {self.safe_height} mm")
        self.get_logger().info(f"Work height (path): {self.prismatic_down_height} mm")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Min distance filter: {self.min_distance_mm} mm")
        self.get_logger().info(f"Home position: ({self.home_x}, {self.home_y}) mm")
        self.get_logger().info(f"Initial position repeats: {self.initial_position_repeats}")

        # ==================== QoS Configuration ====================
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # ==================== Subscribers ====================
        self.create_subscription(PointCloud, 'dxf_pointcloud', self.pointcloud_callback, qos_profile=qos)
        self.create_subscription(String, 'dxf_figures_info', self.figures_info_callback, qos_profile=qos)

        # ==================== Publishers ====================
        self.publisher = self.create_publisher(Twist, 'desired_pos', 10)
        self.rviz_publisher = self.create_publisher(Path, 'planned_trajectory_rviz', 10)

        # ==================== State Variables ====================
        self.all_points = []  # All points from PointCloud
        self.figures_info = []  # Figure metadata
        self.full_trajectory = []  # Complete interpolated trajectory [(x, y, z, type), ...]
        self.trajectory_index = 0  # Current index in trajectory
        self.timer = None
        self.last_published_point = None  # For 1mm filter

        # Calculate time between publications
        self.dt = 1.0 / self.publish_rate

    def pointcloud_callback(self, msg: PointCloud):
        """Receive all waypoints from DXF parser."""
        self.all_points = np.array([[pt.x, pt.y, pt.z] for pt in msg.points])
        self.get_logger().info(f"📥 Received {len(self.all_points)} points in dxf_pointcloud.")

        # If we already have figures_info, start processing
        if self.figures_info:
            self.generate_trajectory()

    def figures_info_callback(self, msg):
        """Receive figure metadata (type and number of points)."""
        try:
            self.figures_info = json.loads(msg.data)
            self.get_logger().info(f"📥 Received figures info: {self.figures_info}")
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing JSON in dxf_figures_info: {e}")
            return

        # If we already have points, start processing
        if len(self.all_points) > 0:
            self.generate_trajectory()

    def generate_trajectory(self):
        """Generate complete interpolated trajectory from all figures."""
        if not self.all_points.size or not self.figures_info:
            return

        self.get_logger().info("🔧 Generating complete trajectory...")
        self.full_trajectory = []
        current_point_idx = 0

        # Get the first point for initial positioning
        first_point = self.all_points[0]
        
        # Repeat the first position multiple times to ensure robot reaches it
        for _ in range(self.initial_position_repeats):
            self.full_trajectory.append((first_point[0], first_point[1], self.safe_height, "INITIAL_POSITION"))
        
        self.get_logger().info(f"📍 Added {self.initial_position_repeats} initial position waypoints at "
                               f"({first_point[0]:.2f}, {first_point[1]:.2f}, {self.safe_height:.2f})")

        for fig_idx, fig in enumerate(self.figures_info):
            num_points = fig["num_points"]
            fig_type = fig["type"]

            # Extract points for this figure
            figure_points = self.all_points[current_point_idx:current_point_idx + num_points]
            current_point_idx += num_points

            # Close the figure by adding the first point at the end (only if not already closed)
            if len(figure_points) > 1:
                # Check if figure is already closed (last point ~= first point)
                first_point = figure_points[0]
                last_point = figure_points[-1]
                distance_to_close = np.sqrt((last_point[0] - first_point[0])**2 + 
                                           (last_point[1] - first_point[1])**2)
                
                # Only add closing point if distance > 0.1mm (figure is not closed)
                if distance_to_close > 0.1:
                    figure_points = np.vstack([figure_points, figure_points[0]])
                    self.get_logger().info(f"   └─ Figure not closed ({distance_to_close:.2f}mm gap), adding closing segment")
                else:
                    self.get_logger().info(f"   └─ Figure already closed (gap: {distance_to_close:.4f}mm), no closing segment needed")

            self.get_logger().info(f"📐 Processing figure {fig_idx + 1}/{len(self.figures_info)} "
                                   f"({fig_type}, {num_points} points)")

            # Determine interpolation settings
            if "CIRCLE" in fig_type or "ARC" in fig_type:
                time_per_segment = self.time_work_segment / 4  # Faster for circles
            else:
                time_per_segment = self.time_work_segment

            # For the first figure, start at safe height and move down
            if fig_idx == 0:
                first_point = figure_points[0]
                # Note: Initial position already added above, just move down now
                # Interpolate down to work height
                down_traj = self._interpolate_segment(
                    first_point[0], first_point[1], self.safe_height,
                    first_point[0], first_point[1], self.prismatic_down_height,
                    self.time_prismatic_down, "INITIAL_DOWN"
                )
                self.full_trajectory.extend(down_traj)

            # Interpolate all segments in this figure
            for i in range(len(figure_points) - 1):
                p1 = figure_points[i]
                p2 = figure_points[i + 1]
                
                # Determine if this is a closing segment
                is_closing = (i == len(figure_points) - 2)
                segment_time = self.time_segment_close if is_closing else time_per_segment
                segment_type = "CLOSE" if is_closing else fig_type

                # Interpolate this segment at work height
                segment_traj = self._interpolate_segment(
                    p1[0], p1[1], self.prismatic_down_height,
                    p2[0], p2[1], self.prismatic_down_height,
                    segment_time, segment_type
                )
                self.full_trajectory.extend(segment_traj)

            # Transition to next figure (if not the last one)
            if fig_idx < len(self.figures_info) - 1:
                # Get last point of current figure and first point of next figure
                last_point = figure_points[-1]
                next_fig_start_idx = current_point_idx
                next_first_point = self.all_points[next_fig_start_idx]

                # Move up
                up_traj = self._interpolate_segment(
                    last_point[0], last_point[1], self.prismatic_down_height,
                    last_point[0], last_point[1], self.safe_height,
                    self.time_prismatic_up, "PRISMATIC_UP"
                )
                self.full_trajectory.extend(up_traj)

                # Move in XY to next figure
                xy_traj = self._interpolate_segment(
                    last_point[0], last_point[1], self.safe_height,
                    next_first_point[0], next_first_point[1], self.safe_height,
                    self.time_xy_transition, "TRANSITION"
                )
                self.full_trajectory.extend(xy_traj)

                # Move down
                down_traj = self._interpolate_segment(
                    next_first_point[0], next_first_point[1], self.safe_height,
                    next_first_point[0], next_first_point[1], self.prismatic_down_height,
                    self.time_prismatic_down, "PRISMATIC_DOWN"
                )
                self.full_trajectory.extend(down_traj)

        # Final move up at the end
        last_point = self.full_trajectory[-1]
        final_up_traj = self._interpolate_segment(
            last_point[0], last_point[1], last_point[2],
            last_point[0], last_point[1], self.safe_height,
            self.time_prismatic_up, "FINAL_UP"
        )
        self.full_trajectory.extend(final_up_traj)

        # Move to home position (x=home_x, y=home_y, z=safe_height)
        # Send home position directly 20 times without interpolation
        self.get_logger().info(f"🏠 Adding home position: ({self.home_x}, {self.home_y}, {self.safe_height}) mm")
        for _ in range(20):
            self.full_trajectory.append((self.home_x, self.home_y, self.safe_height, "HOME"))

        self.get_logger().info(f"✅ Trajectory generation complete: {len(self.full_trajectory)} waypoints")
        self.get_logger().info(f"🏠 Final home position: ({self.home_x}, {self.home_y}, {self.safe_height}) mm")
        
        # Log trajectory breakdown by type
        type_counts = {}
        for _, _, _, seg_type in self.full_trajectory:
            type_counts[seg_type] = type_counts.get(seg_type, 0) + 1
        self.get_logger().info(f"📊 Trajectory breakdown: {type_counts}")

        # Publish full trajectory to RViz
        self.publish_rviz_trajectory()

        # Start publishing at fixed rate
        self.trajectory_index = 0
        self.last_published_point = None
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(self.dt, self.publish_next_waypoint)

    def _interpolate_segment(self, x0, y0, z0, xf, yf, zf, time_duration, segment_type):
        """
        Interpolate between two 3D points over a given time duration.
        Returns a list of (x, y, z, type) tuples.
        """
        num_points = max(2, int(time_duration * self.publish_rate))
        
        x_vals = self._interpolate_1d(x0, xf, time_duration, num_points)
        y_vals = self._interpolate_1d(y0, yf, time_duration, num_points)
        z_vals = self._interpolate_1d(z0, zf, time_duration, num_points)

        return [(x, y, z, segment_type) for x, y, z in zip(x_vals, y_vals, z_vals)]

    def _interpolate_1d(self, q0, qf, T, n_points):
        """1D interpolation based on selected interpolation type."""
        t = np.linspace(0, T, n_points)
        
        if self.interpolation_type == "linear":
            return q0 + (qf - q0) * (t / T)
        
        elif self.interpolation_type == "cubic":
            # Cubic with zero initial and final velocities
            v0, vf = 0.0, 0.0
            a0 = q0
            a1 = v0
            a2 = (3 * (qf - q0) / T**2) - (2 * v0 + vf) / T
            a3 = (2 * (q0 - qf) / T**3) + (v0 + vf) / T**2
            return a0 + a1 * t + a2 * t**2 + a3 * t**3
        
        elif self.interpolation_type == "quintic":
            # Quintic with zero initial and final velocities and accelerations
            v0, vf, a0, af = 0.0, 0.0, 0.0, 0.0
            a0_c = q0
            a1_c = v0
            a2_c = a0 / 2
            a3_c = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T**2) / (2 * T**3)
            a4_c = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T**2) / (2 * T**4)
            a5_c = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T**2) / (2 * T**5)
            return a0_c + a1_c * t + a2_c * t**2 + a3_c * t**3 + a4_c * t**4 + a5_c * t**5
        
        else:
            # Default to linear
            return q0 + (qf - q0) * (t / T)

    def publish_next_waypoint(self):
        """
        Publish the next waypoint in the trajectory at fixed 20Hz rate.
        Implements 1mm minimum distance filter.
        """
        if self.trajectory_index >= len(self.full_trajectory):
            self.get_logger().info("✅ Trajectory execution complete!")
            if self.timer:
                self.timer.cancel()
            return

        current_point = self.full_trajectory[self.trajectory_index]
        x, y, z, segment_type = current_point

        # Always publish critical waypoints and work segments
        # Critical types include initial/final movements and actual work (CIRCLE, POLYLINE, etc.)
        critical_types = ["INITIAL_POSITION", "HOME", "PRISMATIC_UP", "PRISMATIC_DOWN", 
                         "INITIAL_DOWN", "FINAL_UP", "CLOSE", "CIRCLE", "POLYLINE", 
                         "ARC", "LINE", "SPLINE", "LWPOLYLINE"]
        
        should_publish = segment_type in critical_types
        
        # For non-critical types, apply distance filter
        if not should_publish and self.last_published_point is not None:
            dx = x - self.last_published_point[0]
            dy = y - self.last_published_point[1]
            dz = abs(z - self.last_published_point[2])
            distance = math.sqrt(dx**2 + dy**2)
            
            # Publish if movement is significant
            if distance >= self.min_distance_mm or dz >= self.min_distance_mm:
                should_publish = True
        elif self.last_published_point is None:
            # Always publish the first waypoint
            should_publish = True

        if should_publish:
            # Publish to desired_pos
            tw = Twist()
            tw.linear.x = float(x)
            tw.linear.y = float(y)
            tw.linear.z = float(z)
            self.publisher.publish(tw)
            
            self.last_published_point = (x, y, z)
            
            # Log progress
            if self.trajectory_index % 200 == 0 or segment_type in ["INITIAL_POSITION", "HOME", "PRISMATIC_UP", "PRISMATIC_DOWN", "INITIAL_DOWN", "FINAL_UP"]:
                self.get_logger().info(f"📍 Publishing waypoint {self.trajectory_index}/{len(self.full_trajectory)}: "
                                       f"({x:.2f}, {y:.2f}, {z:.2f}) [{segment_type}]")

        self.trajectory_index += 1

    def publish_rviz_trajectory(self):
        """Publish the complete trajectory to RViz for visualization (in meters)."""
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.full_trajectory:
            x, y, z, segment_type = point
            
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
    node = ScaraTrajectoryPlannerV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
