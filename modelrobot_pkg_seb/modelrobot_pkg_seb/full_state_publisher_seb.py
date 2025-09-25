#!/usr/bin/env python3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState # referencia

class FullStatePublisher_seb(Node):
    def __init__(self):
        super().__init__("full_state_publisher_seb")
        # ---- Parameters (declare + get) ----
        self.declare_parameter('joint1', 'arm1_arm2_joint')
        self.declare_parameter('joint2', 'arm2_arm3_joint')
        self.declare_parameter('joint3', 'arm3_arm4_joint')
        self.declare_parameter('joint4', 'arm4_tool_joint')
        
        self.declare_parameter('lwheel_joint', 'chassis_lwheel_joint')
        self.declare_parameter('rwheel_joint', 'chassis_rwheel_joint')
        
        self.declare_parameter('publish_rate', 50.0)      # Hz

        self.joint1 = self.get_parameter('joint1').get_parameter_value().string_value
        self.joint2 = self.get_parameter('joint2').get_parameter_value().string_value
        self.joint3 = self.get_parameter('joint3').get_parameter_value().string_value
        self.joint4 = self.get_parameter('joint4').get_parameter_value().string_value
        
        self.l_joint = self.get_parameter('lwheel_joint').get_parameter_value().string_value
        self.r_joint = self.get_parameter('rwheel_joint').get_parameter_value().string_value
        
        self.rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- State (rad) ----
        self.cmd_theta1 = 0.0
        self.cmd_theta2 = 0.0
        self.cmd_theta3 = 0.0
        self.cmd_theta4 = 0.0
        
        self.cmd_w1 = 0.0
        self.cmd_w2 = 0.0 
        
        # wheel angles (rad)
        self.theta_l = 0.0
        self.theta_r = 0.0

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Quaternion, '/cmd_arm_pos', self._on_cmd_arm_pos, qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Joints: \nArm: {self.joint1}, {self.joint2}, {self.joint3}, {self.joint4}\nWheel: {self.l_joint}, {self.r_joint}\n, rate={self.rate_hz} Hz\n"
        )
        
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_w1 = msg.angular.x
        self.cmd_w2 = msg.angular.y
        
    def _on_cmd_arm_pos(self, msg: Quaternion):
        self.cmd_theta1 = msg.x
        self.cmd_theta2 = msg.y
        self.cmd_theta3 = msg.z
        self.cmd_theta4 = msg.w

    def _on_timer(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # wheel angular velocities
        w_r = self.cmd_w1
        w_l = self.cmd_w2

        # integrate wheel angles
        self.theta_r += w_r * dt
        self.theta_l += w_l * dt

        # Publish joint_states 
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.l_joint, self.r_joint, self.joint1, self.joint2, self.joint3, self.joint4]
        js.position = [self.theta_l, self.theta_r, self.cmd_theta1, self.cmd_theta2, self.cmd_theta3, self.cmd_theta4]
        js.velocity = [w_l, w_r, 0.0, 0.0, 0.0, 0.0]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = FullStatePublisher_seb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()