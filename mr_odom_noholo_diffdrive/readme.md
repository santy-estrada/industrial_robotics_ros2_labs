# Mobile Robotics Odometry Non-Holonomic Differential Drive

## Overview

`mr_odom_noholo_diffdrive` is a ROS 2 package designed for differential-drive mobile robots that calculates odometry based on wheel encoder feedback. The node processes joint states to estimate the robot's position and orientation using a custom kinematic model.

## Features

- Computes robot pose using differential-drive kinematics.
- Publishes odometry information in `nav_msgs/Odometry` format.
- Broadcasts transform between `odom` and `base_link`.
- Includes a teleoperation node for joystick-based control.

## Requirements

- ROS 2 Humble
- Python 3
- `rclpy`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- `numpy`

## Directory Structure

```bash
mr_odom_noholo_diffdrive/
├── launch/
│   ├── joystick.launch.py
│   ├── joystick_wls.launch.py
│   └── robot_spawn.launch.py
├── model/
│   ├── inertia_macro.xacro
│   ├── properties.xacro
│   ├── robot_core.xacro
│   └── robot.urdf.xacro
├── joy_pc_server/
│   └── joystick_server.py
├── config/
│   ├── joystick.yaml
│   └── twist_mux.yaml
├── mr_odom_noholo_diffdrive/
│   ├── __init__.py
│   ├── mr_odom_node_p2.py
│   ├── mr_odom_node_p2_teleop.py
│   └── joy_bridge_node.py
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Clone to a ROS2 existing Workspace

Clone the package into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select mr_odom_noholo_diffdrive
source install/setup.bash
```

Or

## Workspace and Package Creation

```bash
mkdir -p ~/ros2_ws/src
echo ''source ~/ros2_ws/install/setup.bash'' >> ~/.bashrc
source ~/.bashrc
```
In the `src` folder inside of the Workspace
```bash
ros2 pkg create <ros_package_name> --build-type ament_python  --dependencies <package_dependencies>
```

## Building

From the root of the workspace:

```bash
colcon build --packages-select mr_odom_noholo_diffdrive
source install/setup.bash
```

## Usage

### Hard-code Inputs

To run the odometry node:

```bash
ros2 run mr_odom_noholo_diffdrive mr_odom_node_p2
```

To run the Mobile Robot Model:

```bash
ros2 launch mr_odom_noholo_diffdrive robot_spawn.launch.py
```

To run Rviz2:

```bash
rviz2
```

### Teleoperation Input

To run the Mobile Robot Model:

```bash
ros2 launch mr_odom_noholo_diffdrive robot_spawn.launch.py
```

To run Rviz2:

```bash
rviz2
```

To run the teleoperation odometry node:

```bash
ros2 run mr_odom_noholo_diffdrive mr_odom_node_p2_teleop
```
- To run the teleoperation keyboard:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
Or
- To launch the joystick teleoperation node (on Ubuntu machine):

    ```bash
    ros2 launch mr_odom_noholo_diffdrive joystick.launch.py
    ```

    Ensure your joystick is connected and configured as per `config/joystick.yaml`.

- To launch the joystick teleoperation node (on WSL machine):

    ```bash
    ros2 launch mr_odom_noholo_diffdrive joystick_wsl.launch.py
    ```
- On Windows
    - Connect the GamePad to the PC.
    - Copy the folder `joy_pc_server` to Windows.
    - Run the script `joy_pc_server.py`

    **Note**. This script requiere a full installation of Python3.X and PyGame.  


## Other programs recommended to run 

- RViz2 with TF and Robot Model plug-ins
- RQT with Plot topic data


## License

Apache 2.0

## Detail Description `mr_odom_node_p2` 

### Libraries

These are the required libraries:

- `rclpy`: ROS 2 Python client library.
- `numpy`: For numerical operations.
- `tf2_ros`: For broadcasting transforms between frames.
- `geometry_msgs` and `nav_msgs`: Standard ROS 2 message types for transformations and odometry.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
```
### Jacobian Matrix Function

This function returns the Jacobian matrix used to transform velocities from the robot’s body frame to the world frame, depending on its orientation $\varphi$.

```python
def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J
```

### Main Structure

```python
class ODOM_Node_P2(Node):
    def __init__(self):
        super().__init__("odom_node_p2")

    def timer_callback(self):
        pass

    def odometry_publisher(self):
        pass

    def odom_tf_broadcaster(self):
        pass

    def joint_state_publisher(self):
        pass

def main():
    rclpy.init()
    node = ODOM_Node_P2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
### Methods and Content

**Initial Conditions**

This defines:

- $\eta$: Robot pose $[x, y, \varphi]$
- $\theta_{1}, \theta_{2}$: Wheel angular position
- $\omega_{1}, \omega_{2}$: Wheel angular speed
- Vehicle parameters: 
    - $a$: wheel radius
    - $d$: wheel separation to the {B}

```python
# initial conditions
self.eta = np.array([[0.0],
                     [0.0],
                     [0.0]])  

# vehicle parameters
self.a = 0.05  # wheel radius
self.d = 0.2+0.04/2  # distance between wheels and center of the robot
        
# joint states wheel angle
self.th1 = 0.0
self.th2 = 0.0

# angular velocities of the wheels
self.w1 = 1.0
self.w2 = 5.0
```

**Publishers and TF Broadcaster**

- Publishes odometry to the `odom` topic.
- Publishes joint states to the `JointState` topic.
- Broadcasts the transform between `odom` and `base_link`.

```python
# publishers and subscribers
qos = QoSProfile(depth=10)

# odom_publisher
self.odom_pub = self.create_publisher(
    msg_type=Odometry, # <--- CHANGE ME
    topic="odom",
    qos_profile=qos
)

self.joint_state_pub = self.create_publisher(
    msg_type=JointState,
    topic="joint_states",
    qos_profile=qos
)

# tf broadcaster
# Create a TF broadcaster to publish TransformStamped messages onto /tf
# Used to maintain coordinate frame relationships in ROS 2
self.tf_broadcaster = TransformBroadcaster(self)

self.t = TransformStamped()
self.t.header.frame_id = 'odom'
self.t.child_frame_id = 'base_link'
```

**Timer and Update Loop**

Sets a periodic timer with interval `dt = 0.1` seconds to update robot pose.

```python
self.dt = 0.1
self.timer = self.create_timer(self.dt, self.timer_callback)
```

**Main Update Logic**

Each timer tick:
- Transforms Wheel Motion to Body-Frame Motion
- Transforms Body-Frame velocities to World-Frame.
- Integrates pose using Euler method.
- Publishes updated odometry and TF transform.

```python
def timer_callback(self):
        
    psi = self.eta[2,0]
    J = jacobin_matrix(psi)
    
    self.w = np.array([[self.w1],
                    [self.w2]])
    
    # wheel kimematic model
    W = np.array([[self.a/2, self.a/2],
                [0, 0],
                [-self.a/(2*self.d), self.a/(2*self.d)]])
    
    # body velocity vector
    self.xi = W @ self.w
    
    # kinematic model
    eta_dot = J @ self.xi
    
    # euler integration
    self.eta += eta_dot * self.dt
    
    # publish odometry message
    self.odometry_publisher()
    
    # publish tf message
    self.odom_tf_broadcaster()
    
    # publish joint state
    self.joint_state_publisher()

    self.get_logger().info(f"x: {self.eta[0,0]:.2f}, y: {self.eta[1,0]:.2f}, phi: {self.eta[2,0]:.2f}")
```

**Odometry Message Publisher**

Publishes a `nav_msgs/Odometry` message with current position, orientation (as quaternion), and velocities.

```python
def odometry_publisher(self):
    now = self.get_clock().now()
    odom_msg = Odometry() #
    
    # header
    odom_msg.header.stamp = now.to_msg()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    
    # position and orientation
    odom_msg.pose.pose.position.x = self.eta[0,0]
    odom_msg.pose.pose.position.y = self.eta[1,0]
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = np.sin(self.eta[2,0]/2)
    odom_msg.pose.pose.orientation.w = np.cos(self.eta[2,0]/2)
    
    # linear and angular velocities on {body frame}
    odom_msg.twist.twist.linear.x = self.xi[0,0]
    odom_msg.twist.twist.linear.y = self.xi[1,0]
    odom_msg.twist.twist.angular.z = self.xi[2,0]

    # publish the message
    self.odom_pub.publish(odom_msg)
```

**TF Broadcaster**

Broadcasts the current robot transform from `odom → base_link`.

```python
def odom_tf_broadcaster(self):
    # header
    now = self.get_clock().now()
    self.t.header.stamp = now.to_msg()
            
    # position and orientation
    self.t.transform.translation.x = self.eta[0,0]
    self.t.transform.translation.y = self.eta[1,0]
    self.t.transform.translation.z = 0.0
    self.t.transform.rotation.x = 0.0
    self.t.transform.rotation.y = 0.0
    self.t.transform.rotation.z = np.sin(self.eta[2,0]/2)
    self.t.transform.rotation.w = np.cos(self.eta[2,0]/2)
    
    # send the transformation
    self.tf_broadcaster.sendTransform(self.t)
```

**Joint State Message Publisher**

Publish the current state of the robot joint.

```python
def joint_state_publisher(self):
    
    # position integration
    self.th1 += self.w1 * self.dt
    self.th2 += self.w2 * self.dt
    
    # header
    now = self.get_clock().now()
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = now.to_msg()
    joint_state_msg.header.frame_id = "base_link"
    joint_state_msg.name = ["left_wheel_joint", "right_wheel_joint"]

    # position
    joint_state_msg.position = [self.th1, self.th2]
    
    # velocity
    joint_state_msg.velocity = [self.w1, self.w2]
    joint_state_msg.effort = [0.0, 0.0]
    
    # publish the message
    self.joint_state_pub.publish(joint_state_msg)
```

## Author

*Professor*: David Rozo-Osorio, I.M. M.Sc. email: david.rozo31@eia.edu.co

**EIA University**, Mechatronical Eng. - Industrial Robotics