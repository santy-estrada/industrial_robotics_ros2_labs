
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Point32
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import ezdxf
import pathlib
import math
import csv
import json
import os

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node')

    # Parameters
        self.declare_parameter(
            'dxf_file',
            'prueba_7_8.dxf'
        )
        dxf_filename = self.get_parameter('dxf_file').get_parameter_value().string_value
        
        # Get the full path to the DXF file in the installed package
        package_share_dir = get_package_share_directory('scara_pkg_gr03')
        dxf_path = os.path.join(package_share_dir, 'dxf', dxf_filename)
        
        self.get_logger().info(f"Reading DXF file: {dxf_path}")

        # Publishers
        self.path_pub = self.create_publisher(Path, 'dxf_path', 10)

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', qos)
        self.figures_info_pub = self.create_publisher(String, 'dxf_figures_info', qos)

        # Parse and publish
        self.waypoints, self.figures_info = self.parse_dxf(dxf_path)
        self.export_to_csv()
        self.publish_waypoints()
        self.publish_pointcloud()
        self.publish_figures_info()

    def parse_dxf(self, path):
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"File does not exist: {path}")
            return [], []

        doc = ezdxf.readfile(path)
        msp = doc.modelspace()
        waypoints = []
        figures_info = []  # lista de {type, num_points}

        for e in msp:
            points_before = len(waypoints)

            if e.dxftype() == 'LINE':
                waypoints.extend([
                    Point(x=e.dxf.start.x, y=e.dxf.start.y, z=0.0),
                    Point(x=e.dxf.end.x, y=e.dxf.end.y, z=0.0)
                ])
                self.get_logger().info("found line")

            elif e.dxftype() == 'LWPOLYLINE':
                for x, y, *_ in e.get_points():
                    waypoints.append(Point(x=x, y=y, z=0.0))
                self.get_logger().info("found lwpolyline")

            elif e.dxftype() == 'POLYLINE':
                for v in e.vertices:
                    waypoints.append(Point(x=v.dxf.location.x, y=v.dxf.location.y, z=0.0))
                self.get_logger().info("found polyline")

            elif e.dxftype() == 'ARC':
                waypoints.extend(self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    e.dxf.start_angle,
                    e.dxf.end_angle
                ))
                self.get_logger().info("found arc")

            elif e.dxftype() == 'CIRCLE':
                waypoints.extend(self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    0.0,
                    360.0
                ))
                self.get_logger().info("found circle")

            elif e.dxftype() == 'SPLINE':
                for vec in e.flattening(0.5):
                    waypoints.append(Point(x=vec.x, y=vec.y, z=0.0))
                self.get_logger().info("found spline")

            # Calcular puntos agregados en esta figura
            points_after = len(waypoints)
            num_points = points_after - points_before
            if num_points > 0:
                figures_info.append({"type": e.dxftype(), "num_points": num_points})

        self.get_logger().info(f"Extracted {len(waypoints)} waypoints from DXF.")
        return waypoints, figures_info

    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)
        if end_angle < start_angle:
            end_angle += 2 * math.pi

        arc_points = []
        for i in range(num_points + 1):
            angle = start_angle + i * (end_angle - start_angle) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            arc_points.append(Point(x=x, y=y, z=0.0))
        return arc_points

    def publish_waypoints(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = wp
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(path_msg.poses)} poses.")
        self.path_pub.publish(path_msg)

    def publish_pointcloud(self):
        pc = PointCloud()
        pc.header.frame_id = "map"
        pc.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pc.points.append(Point32(x=wp.x, y=wp.y, z=wp.z))

        self.pc_pub.publish(pc)

    def publish_figures_info(self):
        msg = String()
        msg.data = json.dumps(self.figures_info)  # serializar como JSON

        self.get_logger().info("Publishing figures info (JSON):")
        self.get_logger().info(msg.data)
        self.figures_info_pub.publish(msg)

    def export_to_csv(self, filename='dxf_waypoints_v5.csv'):
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'z'])
                for point in self.waypoints:
                    writer.writerow([point.x, point.y, point.z])
            self.get_logger().info(f"Waypoints exported to CSV at: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DXFParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()