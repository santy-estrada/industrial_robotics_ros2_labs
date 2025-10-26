from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mr_mecanum_pos_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name,'model'), glob('model/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidrozoosorio',
    maintainer_email='david.rozo31@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mr_odom_node_p3 = mr_mecanum_pos_ctrl.mr_odom_node_p3:main',
            'mr_odom_node_p3_teleop_wheels = mr_mecanum_pos_ctrl.mr_odom_node_p3_teleop_wheels:main',
            'mr_odom_node_p3_pos_ctrl = mr_mecanum_pos_ctrl.mr_odom_node_p3_pos_ctrl:main',
            'mr_yb_x3_pos_ctrl = mr_mecanum_pos_ctrl.mr_yb_x3_pos_ctrl:main'
        ],
    },
)
