import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro

pkg_robot_folder = 'scara_pkg_gr03'
robot_file = 'ms2r1p_amg.urdf.xacro'
rviz_file = 'rviz_config.rviz'


def generate_launch_description():
    pkg_robot_path = os.path.join(get_package_share_path(pkg_robot_folder))
    default_model_path = os.path.join(pkg_robot_path + '/model/' + robot_file)
    default_rviz_config_path = os.path.join(pkg_robot_path + '/config/' + rviz_file)

    mux_config_path = os.path.join(pkg_robot_path, 'config', 'mux.yaml')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Absolute path to rviz config file')
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file
    robot_description_config = xacro.process_file(default_model_path)

    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    scara_forward_kinematics_node = Node(
        package='scara_pkg_gr03',
        executable='scara_forward_kinematics',
        name='scara_forward_kinematics',
        output='screen',
    )
    
    teleop_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_config_path],
        remappings=[
            ('/cmd_vel_out', '/scara_conf'),
        ]
    )
    
    scara_inverse_kinematics_node = Node(
        package='scara_pkg_gr03',
        executable='scara_inverse_kinematics',
        name='scara_inverse_kinematics',
        output='screen',
    )
    
    scara_goal_pose_translator_node = Node(
        package='scara_pkg_gr03',
        executable='scara_goal_pose_translator',
        name='scara_goal_pose_translator',
        output='screen',
    )
    


    # Launch!
    return LaunchDescription([
        rviz_arg,
        rviz_node,
        gui_arg,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        scara_forward_kinematics_node,
        teleop_twist_mux_node, 
        scara_inverse_kinematics_node,
        scara_goal_pose_translator_node,
    ])