from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pi4_nodes_gr03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
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
            'direct_kinematics_diff=pi4_nodes_gr03.direct_kinematics_diff_node:main',
            'inverse_kinematics_diff=pi4_nodes_gr03.inverse_kinematics_diff_node:main',
            'inverse_kinematics_diff_v2=pi4_nodes_gr03.inverse_kinematics_diff_node_v2:main',
        ],
    },
)
