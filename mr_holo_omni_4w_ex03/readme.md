# Mobile Robotics Odometry Holonomic Omnidirectional Drive

## Overview

`mr_holo_omni_4w_ex03` is a ROS 2 package designed for omnidirectional mobile robots that calculates odometry based on wheel encoder feedback. The node processes joint states to estimate the robot's position and orientation using a custom kinematic model.

## Features

- Computes robot pose using differential-drive kinematics.
- Publishes odometry information in `nav_msgs/Odometry` format.
- Broadcasts transform between `odom` and `base_link`.

## Requirements

- ROS 2 Humble
- Python 3
- `rclpy`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- `numpy`

## Directory Structure

```bash
mr_holo_omni_4w_ex03/
├── launch/
│   └── robot_spawn.launch.py
├── model/
│   ├── inertia_macro.xacro
│   ├── properties.xacro
│   ├── robot_core.xacro (missing or partial develop)
│   └── robot.urdf.xacro
├── mr_holo_omni_4w_ex03/
│   ├── __init__.py
│   └── mr_odom_node_p2.py (missing or partial develop)
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Clone to a ROS2 Workspace

Clone the package into your ROS 2 workspace:

```bash
mkdir -p ros2_ws_ex03/src
cd ~/ros2_ws_ex03/src
git clone <this-repo-url>
cd ~/ros2_ws_ex03
colcon build --packages-select mr_holo_omni_4w --symlink-install 
source install/setup.bash
```

## Usage

### Hard-code Inputs

To run the odometry node:

```bash
ros2 run mr_holo_omni_4w mr_odom_node_p2
```

To run the Mobile Robot Model:

```bash
ros2 launch mr_holo_omni_4w robot_spawn.launch.py
```

To run Rviz2:

```bash
rviz2
```

## License

Apache 2.0


## ROS2 Guide

### Nodes

**Main structure**

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

class NODE(Node):
    def __init__(self):
        super().__init__("NODE")

def main():
    rclpy.init()
    node = NODE()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

**Publisher, Subscriber & timers**

```python
self.create_publisher(<msg_type>,<topic_name>,10)

self.create_subscription(<msg_type>,<topic_name>,<callback_fn>,10)

self.create_timer(<time>, <timer_callback>)
```


### Messages

**Odometry**

```python
from nav_msgs.msg import Odometry

now = self.get_clock().now()
odom_msg = Odometry() #

odom_msg.header.stamp = now.to_msg()
odom_msg.header.frame_id
odom_msg.child_frame_id

odom_msg.pose.pose.position.x
odom_msg.pose.pose.position.y
odom_msg.pose.pose.position.z
odom_msg.pose.pose.orientation.x
odom_msg.pose.pose.orientation.y
odom_msg.pose.pose.orientation.z
odom_msg.pose.pose.orientation.w

odom_msg.twist.twist.linear.x
odom_msg.twist.twist.linear.y
odom_msg.twist.twist.angular.z

odom_pub.publish(odom_msg)
```

**Joint State**

```python
from sensor_msgs.msg import JointState

joint_state_msg = JointState()
joint_state_msg.header.frame_id
joint_state_msg.name

now = self.get_clock().now()
joint_state_msg.header.stamp = now.to_msg()


joint_state_msg.position
joint_state_msg.velocity
joint_state_msg.effort

joint_state_pub.publish(joint_state_msg)
```


**Transformation and TF Broadcaster**

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

tf_broadcaster = TransformBroadcaster(self)

t = TransformStamped()
t.header.frame_id
t.child_frame_id

now = self.get_clock().now()
t.header.stamp = now.to_msg()
        
t.transform.translation.x
t.transform.translation.y
t.transform.translation.z
t.transform.rotation.x
t.transform.rotation.y
t.transform.rotation.z
t.transform.rotation.w

tf_broadcaster.sendTransform(t)
```


### XACRO Guide

**Robot Definition**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
</robot>
```

**Link**

```xml
<link name="chassis">
    <visual>
        <origin xyz="0 0 0"/>
        <geometry>
            <type of body>
        </geometry>   
        <material name="white">
            <color rgba= "1 1 1 1"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <type of body>
        </geometry>
    </collision>
    <inertial>
        <mass value="1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="<I1>" ixy="<I2>" ixz="<I3>" iyy="<I4>" iyz="<I5>" izz="<I6>"/>
    </inertial>
</link>
```

**Type of body**

```xml
<box size="<x0 y0 z0>"/>
<cylinder radius="<d0>" length="<l0>"/>
<sphere radius="<d0>"/>
<mesh filename="package://<pkg/meshes/file_name.stl>" scale="<s0 s1 s2>"/>
```

**Joint**

```xml
<joint name="<joint_name>" type="<joint_type>">
    <parent link="<link_name>"/>
    <child link="<link_name>"/>
    <origin xyz="<location xyz>" rpy="<orientation xyz>"/>
    <axis xyz="<origin>"/>    
</joint>
```
**Note.** the type of the joint can be: `fixed`, `revolute`, `prismatic`, `continuous`


## Author

*Professor*: David Rozo-Osorio, I.M. M.Sc. email: david.rozo31@eia.edu.co

**EIA University**, Mechatronical Eng. - Industrial Robotics