#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState # referencia

class MSP2RStatePublisher_seb(Node):
    def __init__(self):
        super().__init__("msp2r_state_publisher_seb")
        # ---- Parameters (declare + get) ----
        self.declare_parameter('joint1', 'armBase_arm1_joint')
        self.declare_parameter('joint2', 'arm1_arm2_joint')
        self.declare_parameter('publish_rate', 50.0)      # Hz

        self.joint1 = self.get_parameter('joint1').get_parameter_value().string_value
        self.joint2 = self.get_parameter('joint2').get_parameter_value().string_value

        self.rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- State (rad) ----
        self.cmd_theta1 = 0.0
        self.cmd_theta2 = 0.0

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/cmd_arm_pos', self._on_cmd_arm_pos, qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Joints: {self.joint1}, {self.joint2}, rate={self.rate_hz} Hz\n"
        )
        
    def _on_cmd_arm_pos(self, msg: Twist):
        self.cmd_theta1 = msg.linear.y
        self.cmd_theta2 = msg.angular.x

    def _on_timer(self):
        now = self.get_clock().now()

        # Publish joint_states 
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.joint1, self.joint2]
        js.position = [self.cmd_theta1, self.cmd_theta2]
        js.velocity = [0.0, 0.0]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = MSP2RStatePublisher_seb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()