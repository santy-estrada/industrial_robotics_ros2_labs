Joint State Publisher Node in ROS 2
===================================

This document provides a detailed explanation of a ROS 2 node that publishes simulated joint states using sine and cosine functions. 

Code Overview
-------------

The node publishes joint positions to the ``joint_states`` topic, simulating continuous motion for visualization.

Code Walkthrough
----------------

1. **Importing the Required Libraries**

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import math
    import time

- **`rclpy`**: The ROS 2 Python client library.
- **`Node`**: Base class used to create custom ROS nodes.
- **`JointState`**: A message type from the ``sensor_msgs`` package used to send joint positions.
- **`math`**: Provides mathematical functions (e.g., sine and cosine).
- **`time`**: Tracks the time elapsed since the node was started.

2. **Defining the Node Class**

.. code-block:: python

    class JointStatePublisher(Node):
        def __init__(self):
            super().__init__('joint_state_publisher')

The **`JointStatePublisher`** class inherits from the ``Node`` class and initializes a ROS 2 node with the name ``joint_state_publisher``.

3. **Creating the Publisher and Timer**

.. code-block:: python

    self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
    self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

- **`create_publisher()`**: Creates a publisher that sends ``JointState`` messages to the ``joint_states`` topic.
- **`create_timer()`**: Sets a timer to call the ``publish_joint_states`` method every 0.1 seconds (10 Hz).

4. **Initializing the Joint State Message**

.. code-block:: python

    self.joint_state = JointState()
    self.joint_state.name = ['world2odom_joint', 'odom2base_joint', 'joint3']
    self.joint_state.position = [0.0, 0.0, 0.0]
    self.start_time = time.time()

- **`name`**: Defines the joint names.
- **`position`**: Initializes joint positions to ``0.0``.
- **`start_time`**: Records the start time for time-based updates.

5. **Publishing Joint States**

.. code-block:: python

    def publish_joint_states(self):
        current_time = time.time() - self.start_time
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position[0] = math.sin(current_time)
        self.joint_state.position[1] = math.cos(current_time)
        self.joint_state.position[2] = math.sin(2 * current_time)
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Publishing: {self.joint_state.position}')

- **`current_time`**: Time elapsed since the node started.
- **`header.stamp`**: Updates the message timestamp.
- **`position`**: Updates joint positions using sine and cosine waves.
- **`publish()`**: Publishes the updated joint state.
- **`get_logger().info()`**: Logs the current joint positions.

6. **Main Function and Lifecycle Management**

.. code-block:: python

    def main(args=None):
        rclpy.init(args=args)
        node = JointStatePublisher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

- **`rclpy.init()`**: Initializes the ROS 2 communication layer.
- **`rclpy.spin()`**: Keeps the node active.
- **`KeyboardInterrupt`**: Handles clean shutdown with ``Ctrl+C``.

7. **Entry Point**

.. code-block:: python

    if __name__ == '__main__':
        main()

This ensures the script runs the ``main()`` function if executed directly.

Summary
-------

This ROS 2 node publishes simulated joint states using **sine** and **cosine** waves. The published data can be used to visualize joint movements on a robot. It publishes on the ``joint_states`` topic every 0.1 seconds, with:
- **Names**: `world2odom_joint`, `odom2base_joint`, and `joint3`
- **Positions**: Updated dynamically using sine and cosine functions
- **Timestamp**: Current time when the message was generated

The **`main()`** function ensures the node is initialized, runs continuously, and shuts down cleanly upon interruption.

