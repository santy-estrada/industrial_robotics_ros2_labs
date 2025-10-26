from setuptools import find_packages, setup

package_name = 'mr_odom_pkg_class_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mr_odom_node_p1 = mr_odom_pkg_class_pkg.mr_odom_node_p1:main',
        ],
    },
)
