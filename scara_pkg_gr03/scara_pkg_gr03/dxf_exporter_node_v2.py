#!/usr/bin/env python3
"""
DXF Exporter Node V2
====================
This node reads DXF files and exports waypoints with advanced features:
- Workspace centering: Move the DXF geometric center to a specified workspace location
- Scaling: Scale the entire piece relative to its center
- Entity sorting: Optimize path by sorting entities using nearest-neighbor algorithm
- QoS configuration: TRANSIENT_LOCAL + RELIABLE for compatibility with trajectory planner

Topics:
-------
Publishers:
    - dxf_pointcloud (PointCloud): All waypoints for visualization
    - dxf_figures_info (String): JSON metadata about figures (type, num_points)

Parameters:
-----------
    - dxf_file (str): Path to DXF file to parse
    - workspace_center_x (float): Target X coordinate for piece center (mm)
    - workspace_center_y (float): Target Y coordinate for piece center (mm)
    - scale (float): Scale factor (1.0 = 100%, 0.5 = 50%, 2.0 = 200%)

Usage Example:
--------------
    ros2 run scara_pkg_gr03 dxf_exporter_node_v2 --ros-args \\
        -p dxf_file:=/path/to/file.dxf \\
        -p workspace_center_x:=200.0 \\
        -p workspace_center_y:=0.0 \\
        -p scale:=0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import ezdxf
import pathlib
import math
import json
import os


class DXFExporterNodeV2(Node):
    def __init__(self):
        super().__init__('dxf_exporter_node_v2')

        # ==================== Parameters ====================
        self.declare_parameter('dxf_file', 'prueba_7_8.dxf')
        self.declare_parameter('workspace_center_x', 200.0)
        self.declare_parameter('workspace_center_y', 0.0)
        self.declare_parameter('scale', 1.0)

        dxf_filename = self.get_parameter('dxf_file').value
        self.workspace_center_x = self.get_parameter('workspace_center_x').value
        self.workspace_center_y = self.get_parameter('workspace_center_y').value
        self.scale = self.get_parameter('scale').value

        # Get the full path to the DXF file
        if os.path.isabs(dxf_filename):
            dxf_path = dxf_filename
        else:
            # Try to find it in the package share directory
            try:
                package_share_dir = get_package_share_directory('scara_pkg_gr03')
                dxf_path = os.path.join(package_share_dir, 'dxf', dxf_filename)
            except:
                # If package not found, use as relative path
                dxf_path = dxf_filename

        self.get_logger().info(f"=== DXF Exporter Node V2 Configuration ===")
        self.get_logger().info(f"DXF file: {dxf_path}")
        self.get_logger().info(f"Target workspace center: ({self.workspace_center_x}, {self.workspace_center_y}) mm")
        self.get_logger().info(f"Scale factor: {self.scale}")

        # ==================== QoS Configuration ====================
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # ==================== Publishers ====================
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', qos)
        self.figures_info_pub = self.create_publisher(String, 'dxf_figures_info', qos)

        # ==================== Process DXF ====================
        self.all_entities = []  # List of (points, entity_type, entity_id)
        self.parse_dxf(dxf_path)
        
        if self.all_entities:
            self.apply_transformations()
            self.sort_entities()
            self.publish_data()
        else:
            self.get_logger().error("❌ No entities found in DXF file!")

    def parse_dxf(self, path):
        """Parse DXF file and extract entities with their points."""
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"❌ File does not exist: {path}")
            return

        try:
            doc = ezdxf.readfile(path)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to read DXF file: {e}")
            return

        msp = doc.modelspace()
        entity_id_counter = 0

        self.get_logger().info("📄 Parsing DXF entities...")

        for e in msp:
            entity_type = e.dxftype()
            current_entity_points = []

            # Extract points based on entity type
            if entity_type == 'LINE':
                current_entity_points.append((e.dxf.start.x, e.dxf.start.y, 0.0))
                current_entity_points.append((e.dxf.end.x, e.dxf.end.y, 0.0))

            elif entity_type == 'LWPOLYLINE':
                points = list(e.get_points())
                for x, y, *_ in points:
                    current_entity_points.append((x, y, 0.0))
                # Close if needed
                if e.is_closed and len(points) > 0:
                    first_point = points[0]
                    current_entity_points.append((first_point[0], first_point[1], 0.0))

            elif entity_type == 'POLYLINE':
                points = list(e.points())
                for p in points:
                    current_entity_points.append((p.x, p.y, 0.0))
                if e.is_closed and len(points) > 0:
                    first_point = points[0]
                    current_entity_points.append((first_point.x, first_point.y, 0.0))

            elif entity_type == 'ARC':
                arc_points = self._approximate_arc(
                    e.dxf.center.x, e.dxf.center.y, e.dxf.radius,
                    e.dxf.start_angle, e.dxf.end_angle
                )
                for point in arc_points:
                    current_entity_points.append((point.x, point.y, point.z))

            elif entity_type == 'CIRCLE':
                circle_points = self._approximate_arc(
                    e.dxf.center.x, e.dxf.center.y, e.dxf.radius, 0.0, 360.0
                )
                for point in circle_points:
                    current_entity_points.append((point.x, point.y, point.z))
                # Close the circle
                if len(circle_points) > 0:
                    first_point = circle_points[0]
                    current_entity_points.append((first_point.x, first_point.y, first_point.z))

            elif entity_type == 'SPLINE':
                for vec in e.flattening(0.5):
                    current_entity_points.append((vec.x, vec.y, 0.0))

            else:
                self.get_logger().debug(f"Skipping unsupported entity type: {entity_type}")
                continue

            # Store entity if it has points
            if current_entity_points:
                self.all_entities.append({
                    'points': current_entity_points,
                    'type': entity_type,
                    'id': entity_id_counter
                })
                entity_id_counter += 1
                self.get_logger().info(f"  ✓ {entity_type}: {len(current_entity_points)} points")

        self.get_logger().info(f"📊 Parsed {len(self.all_entities)} entities from DXF")

    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        """Approximate an arc or circle with discrete points."""
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        # Handle wrapping (e.g., 350° to 10°)
        if end_angle < start_angle:
            end_angle += 2 * math.pi

        arc_points = []
        for i in range(num_points + 1):
            angle = start_angle + i * (end_angle - start_angle) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            arc_points.append(Point(x=x, y=y, z=0.0))
        return arc_points

    def apply_transformations(self):
        """Apply scaling and centering transformations to all entities."""
        if not self.all_entities:
            return

        # Step 1: Calculate original geometric center
        all_points = []
        for entity in self.all_entities:
            all_points.extend(entity['points'])

        if not all_points:
            return

        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)

        original_center_x = min_x + (max_x - min_x) / 2
        original_center_y = min_y + (max_y - min_y) / 2

        self.get_logger().info(f"📍 Original DXF center: ({original_center_x:.2f}, {original_center_y:.2f}) mm")
        self.get_logger().info(f"📏 Original bounding box: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")

        # Step 2: Apply scaling relative to original center
        if self.scale != 1.0:
            self.get_logger().info(f"🔍 Applying scale factor: {self.scale}")
            for entity in self.all_entities:
                scaled_points = []
                for p in entity['points']:
                    # Translate to origin
                    dx = p[0] - original_center_x
                    dy = p[1] - original_center_y
                    # Scale
                    dx_scaled = dx * self.scale
                    dy_scaled = dy * self.scale
                    # Translate back
                    x_scaled = dx_scaled + original_center_x
                    y_scaled = dy_scaled + original_center_y
                    scaled_points.append((x_scaled, y_scaled, p[2]))
                entity['points'] = scaled_points

        # Step 3: Recalculate center after scaling
        all_points = []
        for entity in self.all_entities:
            all_points.extend(entity['points'])

        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)

        current_center_x = min_x + (max_x - min_x) / 2
        current_center_y = min_y + (max_y - min_y) / 2

        # Step 4: Calculate offset to target center
        offset_x = self.workspace_center_x - current_center_x
        offset_y = self.workspace_center_y - current_center_y

        self.get_logger().info(f"↗️  Translation offset: ({offset_x:.2f}, {offset_y:.2f}) mm")

        # Step 5: Apply translation
        for entity in self.all_entities:
            translated_points = []
            for p in entity['points']:
                translated_points.append((p[0] + offset_x, p[1] + offset_y, p[2]))
            entity['points'] = translated_points

        # Final verification
        all_points = []
        for entity in self.all_entities:
            all_points.extend(entity['points'])

        final_min_x = min(p[0] for p in all_points)
        final_max_x = max(p[0] for p in all_points)
        final_min_y = min(p[1] for p in all_points)
        final_max_y = max(p[1] for p in all_points)
        final_center_x = final_min_x + (final_max_x - final_min_x) / 2
        final_center_y = final_min_y + (final_max_y - final_min_y) / 2

        self.get_logger().info(f"✅ Final center: ({final_center_x:.2f}, {final_center_y:.2f}) mm")
        self.get_logger().info(f"✅ Final bounding box: X=[{final_min_x:.2f}, {final_max_x:.2f}], "
                               f"Y=[{final_min_y:.2f}, {final_max_y:.2f}]")

    def sort_entities(self):
        """Sort entities using nearest-neighbor algorithm to minimize transitions."""
        if len(self.all_entities) <= 1:
            return

        self.get_logger().info("🔄 Sorting entities using nearest-neighbor algorithm...")

        def calculate_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

        sorted_entities = []
        remaining_entities = self.all_entities.copy()

        # Start with the first entity
        sorted_entities.append(remaining_entities.pop(0))

        # Iteratively find the nearest next entity
        while remaining_entities:
            last_point = sorted_entities[-1]['points'][-1]  # Last point of last entity
            best_match_idx = -1
            min_dist = float('inf')
            reverse_path = False

            for i, entity in enumerate(remaining_entities):
                # Check distance to start of entity
                dist_to_start = calculate_distance(last_point, entity['points'][0])
                if dist_to_start < min_dist:
                    min_dist = dist_to_start
                    best_match_idx = i
                    reverse_path = False

                # Check distance to end of entity (consider reversing)
                dist_to_end = calculate_distance(last_point, entity['points'][-1])
                if dist_to_end < min_dist:
                    min_dist = dist_to_end
                    best_match_idx = i
                    reverse_path = True

            if best_match_idx != -1:
                next_entity = remaining_entities.pop(best_match_idx)
                
                # Reverse path if end is closer
                if reverse_path:
                    next_entity['points'].reverse()
                    self.get_logger().debug(f"  Reversed entity {next_entity['id']} (end was closer)")

                sorted_entities.append(next_entity)
                self.get_logger().debug(f"  Added entity {next_entity['id']}, "
                                        f"distance from previous: {min_dist:.2f} mm")
            else:
                break

        self.all_entities = sorted_entities
        self.get_logger().info(f"✅ Entities sorted: {len(sorted_entities)} entities")

    def publish_data(self):
        """Publish PointCloud and figure metadata."""
        # Prepare data structures
        all_points = []
        figures_info = []

        for entity in self.all_entities:
            # Add points to flat list
            all_points.extend(entity['points'])
            
            # Add metadata
            figures_info.append({
                'type': entity['type'],
                'num_points': len(entity['points'])
            })

        # Publish PointCloud
        pc = PointCloud()
        pc.header.frame_id = "map"
        pc.header.stamp = self.get_clock().now().to_msg()

        for p in all_points:
            pc.points.append(Point32(x=p[0], y=p[1], z=p[2]))

        self.pc_pub.publish(pc)
        self.get_logger().info(f"📤 Published PointCloud: {len(pc.points)} points")

        # Publish figures info as JSON
        msg = String()
        msg.data = json.dumps(figures_info)
        self.figures_info_pub.publish(msg)
        self.get_logger().info(f"📤 Published figures info: {len(figures_info)} figures")
        self.get_logger().info(f"   Details: {figures_info}")


def main(args=None):
    rclpy.init(args=args)
    node = DXFExporterNodeV2()
    
    # Keep node alive to maintain TRANSIENT_LOCAL messages
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
