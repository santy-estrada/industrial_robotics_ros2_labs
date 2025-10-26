import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='rov_pkg_gr03' #<--- CHANGE ME

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='rov_pkg_gr03', 
                    executable='joy_bridge_node'
    )

    teleop_node_joy = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    rov_converter_node = Node(package=package_name,
                              executable='rov_converter_node', # El script que acabamos de crear
                              name='rov_converter_node',)
                              # Se suscribe a /cmd_vel_joy y publica en /rov_motor_control
    

    # Launch them all!
    return LaunchDescription([
        joy_node,
        teleop_node_joy,
        #twist_mux_node
        rov_converter_node
    ])