#!/usr/bin/env python3
"""
Offline Processing Test Launch File
====================================
This launch file runs the offline trajectory processing pipeline using
programmatically generated test trajectories instead of DXF files.

Pipeline:
1. test_trajectory_generator: Generates geometric shapes (circle, square, star, etc.)
2. offline_trajectory_generator: Interpolates waypoints
3. offline_inverse_kinematics: Computes joint angles for all waypoints
4. joint_trajectory_interpolator: Interpolates between joint configurations

Usage:
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py

    # Use specific shape:
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=circle
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=square
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=star
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=spiral
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=lissajous

    # Override parameters:
    ros2 launch scara_pkg_gr03 offline_processing_test.launch.py shape_type:=circle radius:=75.0
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('scara_pkg_gr03')
    
    # Config files
    test_params = os.path.join(pkg_dir, 'config', 'test_trajectory_params.yaml')
    offline_params = os.path.join(pkg_dir, 'config', 'offline_trajectory_params.yaml')
    scara_params = os.path.join(pkg_dir, 'config', 'scara_kinematics_params.yaml')
    
    # Launch arguments for easy parameter overrides
    shape_type_arg = DeclareLaunchArgument(
        'shape_type',
        default_value='circle',
        description='Shape to generate: circle, square, star, spiral, lissajous'
    )
    
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='50.0',
        description='Radius for circle/square/star/spiral (mm)'
    )
    
    num_points_arg = DeclareLaunchArgument(
        'num_points',
        default_value='100',
        description='Number of points to generate'
    )
    
    # Nodes
    test_gen_node = Node(
        package='scara_pkg_gr03',
        executable='test_trajectory_generator',
        name='test_trajectory_generator',
        output='screen',
        parameters=[
            test_params,
            {
                'shape_type': LaunchConfiguration('shape_type'),
                'radius': LaunchConfiguration('radius'),
                'num_points': LaunchConfiguration('num_points'),
            }
        ]
    )
    
    offline_traj_node = Node(
        package='scara_pkg_gr03',
        executable='offline_trajectory_generator',
        name='offline_trajectory_generator',
        output='screen',
        parameters=[offline_params]
    )
    
    offline_ik_node = Node(
        package='scara_pkg_gr03',
        executable='offline_inverse_kinematics',
        name='offline_inverse_kinematics',
        output='screen',
        parameters=[scara_params]
    )
    
    joint_interp_node = Node(
        package='scara_pkg_gr03',
        executable='joint_trajectory_interpolator',
        name='joint_trajectory_interpolator',
        output='screen',
        parameters=[offline_params]
    )
    
    return LaunchDescription([
        shape_type_arg,
        radius_arg,
        num_points_arg,
        test_gen_node,
        offline_traj_node,
        offline_ik_node,
        joint_interp_node,
    ])
