Using the SCARA Package
========================

This document provides instructions on how to clone and use the ``scara_pkg_gr03`` package from a ROS 2 workspace.

Cloning the Package
-------------------

Since ``scara_pkg_gr03`` is a folder within a larger repository, use Git's sparse checkout to clone only this specific folder. Follow these steps:

1. Initialize a new Git repository in your desired directory (replace ``<repo>`` with your repository name - any local name - and ``<url>`` with the repository URL https://github.com/santy-estrada/industrial_robotics_ros2_labs.git):

    .. code-block:: bash

        git init <repo>
        cd <repo>
        git remote add -f origin <url>

2. Enable sparse checkout and specify the folder to clone (note: the main branch is ``main``, not ``master``):

    .. code-block:: bash

        git config core.sparseCheckout true
        echo "scara_pkg_gr03/" >> .git/info/sparse-checkout
        git pull origin main

This will clone only the ``scara_pkg_gr03`` folder into your workspace.

Running the Package
-------------------

After cloning, ensure you are in a ROS 2 workspace (e.g., ``/home/rozo/ros2_ws_2502``) and build the package:

.. code-block:: bash

    colcon build --symlink-install

To run the package:

1. Launch the main application in the first terminal:

    .. code-block:: bash

        ros2 launch scara_pkg_gr03 scara_p3.launch.py

2. In a second terminal, run the DXF exporter node:

    .. code-block:: bash

        ros2 run scara_pkg_gr03 scara_dxf_exporter_node

3. In a third terminal, run the trajectory planner:

    .. code-block:: bash

        ros2 run scara_pkg_gr03 scara_trajectory_planner

The trajectory animation will be visible in the RViz tab opened by the launch file.

Configuration Note
------------------

Ensure the correct topic is set in ``scara_forward_kinematics.py`` for proper functionality:

.. code-block:: python

    self.create_subscription(Twist, '/scara_conf', self._scara_configuration, qos)  # Connect to scara_conf if there is no pico