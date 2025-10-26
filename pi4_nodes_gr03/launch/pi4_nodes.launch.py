import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nombre del paquete donde se encuentran los nodos
    pkg_name = 'pi4_nodes_gr03'
    
    # 1. Nodo de Cinemática Inversa (v2)
    # IMPORTANTE: El 'executable' debe coincidir con el alias de 'console_scripts' en setup.py.
    inverse_kinematics_v2_node = Node(
        package=pkg_name,
        executable='inverse_kinematics_diff_v2', # <-- CORREGIDO: Usando el alias de setup.py
        name='inverse_kinematics_v2',
        output='screen',
    )

    # 2. Nodo de Cinemática Directa
    # IMPORTANTE: El 'executable' debe coincidir con el alias de 'console_scripts' en setup.py.
    direct_kinematics_node = Node(
        package=pkg_name,
        executable='direct_kinematics_diff', # <-- CORREGIDO: Usando el alias de setup.py
        name='direct_kinematics',
        output='screen',
    )
    
    return LaunchDescription([
        inverse_kinematics_v2_node,
        direct_kinematics_node
    ])