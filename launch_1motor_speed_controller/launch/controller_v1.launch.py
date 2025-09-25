from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

pkg_folder = 'launch_1motor_speed_controller'
config_file = 'mux.yaml'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    mux_config_path = os.path.join(pkg_path, 'config', config_file)

    teleop_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_config_path],
        remappings=[
            ('/cmd_vel_out', '/cmd')
        ]
    )



    return LaunchDescription([
        teleop_twist_mux_node
    ])
