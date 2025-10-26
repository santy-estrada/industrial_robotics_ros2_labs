from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'scara_pkg_gr03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'model'), glob("model/*.*")),
        (os.path.join("share", package_name,'config'), glob("config/*.*")),
        (os.path.join("share", package_name,'dxf'), glob("dxf/*.*")),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santy',
    maintainer_email='santiago.estrada6@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scara_forward_kinematics= scara_pkg_gr03.scara_forward_kinematics:main',
            'scara_inverse_kinematics= scara_pkg_gr03.scara_inverse_kinematics:main',
            'scara_trajectory_planner= scara_pkg_gr03.scara_trajectory_planner:main',
            'scara_goal_pose_translator= scara_pkg_gr03.scara_goal_pose_translator:main',
            'scara_dxf_exporter_node= scara_pkg_gr03.dxf_exporter_node:main',
        ],
    },
)
