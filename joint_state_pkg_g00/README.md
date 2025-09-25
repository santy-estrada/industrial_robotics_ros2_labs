# Joint State Node with Launch Configuration and Visualization

This repository provides a complete setup for launching a Joint State Node along with associated tools for robot visualization and state publishing. The configuration uses ROS (Robot Operating System) to process a URDF/XACRO file, publish the robot's state, and visualize the robot in RViz.

## Repository Overview
The repository contains the following essential components:

1. **Joint State Node**: A ROS node that publishes the current joint states of the robot. This is used to track joint positions, velocities, and efforts in real-time.

2. **Launch File**: The launch file coordinates the execution of several nodes and tools, including:

    - [The Joint State Node](https://github.com/ddrozo31/joint_state_pkg_g00/blob/main/joint_state_publisher_node.rst)
    - RViz for visualization
    - robot_state_publisher to broadcast the robot's URDF model
    - Xacro processing to generate the URDF dynamically

3. **Xacro/URDF Files**: The robot model is defined using Xacro, which allows for parameterized URDF descriptions. These files are processed during launch to generate a valid URDF for the robot.


### Setup Instructions

1. Clone the repository

```bash

git clone https://github.com/ddrozo31/joint_state_pkg_g00.git
```

2. Ensure dependencies are installed

Make sure the following ROS packages are installed:

    - robot_state_publisher
    - joint state node
    - xacro
    - rviz

3. Build the workspace and source

If your node is inside a ROS workspace, ensure you build it:

```bash

colcon build --symlink-install
source ~/ros2_ws_2402/install/setup.bash
```

4. Run the launch file:

```bash
ros2 launch joint_state_pkg_g00 rsp.launch.py

```

5. Configure RViz2:

    - Select the `world`.
    - Add a TF2 plugin, include the names of the topics.

## Presentation

**EIA University**, Mechatronics Eng. - Industrial Robotics Laboratory 
*Professor*: David Rozo Osorio M.Sc. email: david.rozo31@eia.edu.co
