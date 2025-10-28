from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Offline Processing Launch File
    ==============================
    This launch file runs the offline trajectory processing pipeline:
    1. DXF Exporter - reads DXF file and publishes waypoints
    2. Offline Trajectory Generator - republishes all waypoints at once
    3. Offline Inverse Kinematics - computes joint configurations for all waypoints
    4. Joint Trajectory Interpolator - interpolates J1 and J2 angles
    
    The output is a complete interpolated trajectory ready for execution.
    """
    
    # Package directories
    pkg_dir = get_package_share_directory('scara_pkg_gr03')
    
    # Parameter files
    scara_params = os.path.join(pkg_dir, 'config', 'scara_kinematics_params.yaml')
    dxf_params = os.path.join(pkg_dir, 'config', 'scara_v2_params.yaml')
    interpolator_params = os.path.join(pkg_dir, 'config', 'offline_trajectory_params.yaml')
    
    # Nodes
    dxf_exporter = Node(
        package='scara_pkg_gr03',
        executable='dxf_exporter_node_v2',
        name='dxf_exporter_node_v2',
        parameters=[dxf_params],
        output='screen'
    )
    
    trajectory_generator = Node(
        package='scara_pkg_gr03',
        executable='offline_trajectory_generator',
        name='offline_trajectory_generator',
        parameters=[interpolator_params],
        output='screen'
    )
    
    offline_ik = Node(
        package='scara_pkg_gr03',
        executable='offline_inverse_kinematics',
        name='offline_inverse_kinematics',
        parameters=[scara_params],
        output='screen'
    )
    
    interpolator = Node(
        package='scara_pkg_gr03',
        executable='joint_trajectory_interpolator',
        name='joint_trajectory_interpolator',
        parameters=[interpolator_params],
        output='screen'
    )
    
    return LaunchDescription([
        dxf_exporter,
        trajectory_generator,
        offline_ik,
        interpolator
    ])
