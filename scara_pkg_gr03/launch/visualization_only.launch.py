from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    """
    Visualization Only Launch File
    ==============================
    This launch file runs only the visualization nodes without executing
    any trajectory on the robot.
    
    Nodes:
    - Robot State Publisher: Publishes robot URDF
    - Forward Kinematics: Visualizes robot state and end effector path
    - RViz: 3D visualization
    
    Use this to visualize the robot state while trajectory execution
    is happening, or to review recorded data.
    """
    
    # Package directories
    pkg_dir = get_package_share_directory('scara_pkg_gr03')
    
    # Parameter and config files
    scara_params = os.path.join(pkg_dir, 'config', 'scara_kinematics_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    
    # Robot model
    robot_file = os.path.join(pkg_dir, 'model', 'ms2r1p_amg.urdf.xacro')
    robot_description_config = xacro.process_file(robot_file)
    robot_params = {'robot_description': robot_description_config.toxml()}
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_params]
    )
    
    forward_kinematics = Node(
        package='scara_pkg_gr03',
        executable='scara_forward_kinematics',
        name='scara_forward_kinematics',
        parameters=[scara_params],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        forward_kinematics,
        rviz
    ])
