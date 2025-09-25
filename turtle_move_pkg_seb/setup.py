from setuptools import find_packages, setup

package_name = 'turtle_move_pkg_seb'

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
    maintainer_email='santy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # terminal_name = pkg_name.node_file_name:main
            'turtle_move_node_seb = turtle_move_pkg_seb.turtle_move_node_seb:main'
        ],
    },
)
