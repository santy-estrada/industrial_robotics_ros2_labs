from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rov_pkg_gr03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'config'), glob("config/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejo',
    maintainer_email='alejo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_bridge_node=diff_robot_gr03.joy_bridge_node:main',
            'rov_converter_node=rov_pkg_gr03.rov_converter_node:main',
            'rov_gui_node=rov_pkg_gr03.rov_gui_node:main',
            'rov_teleop_node=rov_pkg_gr03.rov_teleop_node:main',
        ],
    },
)
