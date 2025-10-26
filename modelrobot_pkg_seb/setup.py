from setuptools import find_packages, setup

from glob import glob
import os

package_name = 'modelrobot_pkg_seb'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santy',
    maintainer_email='santy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_state_publisher_seb= modelrobot_pkg_seb.diff_drive_state_publisher_seb:main',
            'arm_state_publisher_seb= modelrobot_pkg_seb.arm_state_publisher_seb:main',
            'full_state_publisher_seb= modelrobot_pkg_seb.full_state_publisher_seb:main',
        ],
    },
)
