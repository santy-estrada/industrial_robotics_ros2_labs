from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Trajectory Execution Launch File
    ================================
    This launch file executes a pre-computed trajectory by publishing
    configurations to the Pico at a controlled rate.
    
    This is part of the OFFLINE trajectory planning system and includes
    the mux and forward kinematics (DO NOT use with digital_twin_v2).
    
    Prerequisites:
    - Run offline_processing.launch.py first to generate the trajectory
    - The interpolated trajectory must be available on the /interpolated_trajectory topic
    
    System Architecture:
    -------------------
    Online:  digital_twin_v2 + dxf_exporter_v2
    Offline: offline_processing + visualization_only + trajectory_execution
    """
    
    # Package directories
    pkg_dir = get_package_share_directory('scara_pkg_gr03')
    
    # Parameter files
    executor_params = os.path.join(pkg_dir, 'config', 'trajectory_executor_params.yaml')
    mux_config_path = os.path.join(pkg_dir, 'config', 'mux.yaml')
    
    # Nodes
    executor = Node(
        package='scara_pkg_gr03',
        executable='trajectory_executor',
        name='trajectory_executor',
        parameters=[executor_params],
        output='screen'
    )
    
    # Twist Mux (for command multiplexing)
    # This routes /inv_kin -> /scara_conf for the Pico
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_config_path],
        remappings=[
            ('/cmd_vel_out', '/scara_conf'),
        ]
    )
    
    return LaunchDescription([
        twist_mux,
        executor
    ])
