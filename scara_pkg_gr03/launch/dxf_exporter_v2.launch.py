#!/usr/bin/env python3
"""
DXF Exporter V2 + Trajectory Planner Launch File
=================================================
This launch file starts the DXF exporter node V2 and the trajectory planner V2.

The workflow:
1. DXF Exporter parses the DXF file
2. Applies scaling and centering transformations
3. Sorts entities for optimal path
4. Publishes PointCloud and figure metadata with TRANSIENT_LOCAL QoS
5. Trajectory Planner receives the data and generates trajectory
6. Publishes waypoints to /desired_pos at 20Hz with 1mm filtering

Usage:
------
Launch this AFTER launching the SCARA digital twin:
1. First: ros2 launch scara_pkg_gr03 scara_digital_twin_v2.launch.py
2. Then:  ros2 launch scara_pkg_gr03 dxf_exporter_v2.launch.py

To override parameters:
ros2 launch scara_pkg_gr03 dxf_exporter_v2.launch.py dxf_file:=my_file.dxf scale:=0.5
"""

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

pkg_name = 'scara_pkg_gr03'


def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_path(pkg_name)
    
    # Path to configuration file
    config_file_path = os.path.join(pkg_path, 'config', 'scara_v2_params.yaml')

    # DXF Exporter Node V2
    dxf_exporter_node = Node(
        package=pkg_name,
        executable='dxf_exporter_node_v2',
        name='dxf_exporter_node_v2',
        output='screen',
        parameters=[config_file_path],
        emulate_tty=True,
    )

    # Trajectory Planner V2 Node
    trajectory_planner_node = Node(
        package=pkg_name,
        executable='scara_trajectory_planner_v2',
        name='scara_trajectory_planner_v2',
        output='screen',
        parameters=[config_file_path],
        emulate_tty=True,
    )

    return LaunchDescription([
        dxf_exporter_node,
        trajectory_planner_node,
    ])
