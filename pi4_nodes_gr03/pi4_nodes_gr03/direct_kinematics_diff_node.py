#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

def jacobin_matrix(phi):
    return np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [0,            0,           1]
    ])

class Direct_Kinematics_Diff_Node(Node):
    def __init__(self):
        super().__init__("direct_kinematics_node")

        # --- Estado inicial ---
        self.eta = np.zeros((3, 1))            # Pose real
        self.eta_expected = np.zeros((3, 1))   # Pose ideal
        self.xi = np.zeros((3, 1))
        self.xi_expected = np.zeros((3, 1))

        self.w1 = self.w2 = 0.0
        self.w1_expected = self.w2_expected = 0.0

        # Parámetros físicos
        self.a = 0.042 / 2
        self.d = 0.12766 / 2 + 0.019 / 2

        # Variables de ruedas
        self.th1_real = 0.0
        self.th2_real = 0.0
        self.th1_ideal = 0.0
        self.th2_ideal = 0.0

        # --- Subscriptores ---
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, "diff_measurements", self.cmd_vel_callback, qos)

        # 🆕 Ahora el setpoint también es tipo Twist
        self.create_subscription(Twist, "wheel_setpoint", self.wheel_setpoint_callback, qos)

        # --- Publicadores ---
        self.odom_real_pub = self.create_publisher(Odometry, "odom_real", qos)
        self.odom_ideal_pub = self.create_publisher(Odometry, "odom_ideal", qos)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", qos)
        self.error_pub = self.create_publisher(Float32MultiArray, "pose_error", qos)

        # --- TF broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.dt = 0.1
        self.create_timer(self.dt, self.timer_callback)

    # === Callbacks de suscripción ===
    def cmd_vel_callback(self, msg):
        # Medidas reales (RPM → rad/s)
        self.w1 = msg.linear.x * 2 * np.pi / 60 / 4
        self.w2 = msg.angular.z * 2 * np.pi / 60 / 4

    def wheel_setpoint_callback(self, msg):
        # 🆕 Setpoint ideal (RPM → rad/s)
        self.w1_expected = msg.linear.x * 2 * np.pi / 60
        self.w2_expected = msg.angular.z * 2 * np.pi / 60

    # === Cálculo cinemático ===
    def compute_pose(self, w1, w2, eta):
        psi = eta[2, 0]
        J = jacobin_matrix(psi)
        W = np.array([
            [self.a / 2, self.a / 2],
            [0, 0],
            [-self.a / (2 * self.d), self.a / (2 * self.d)]
        ])
        xi = W @ np.array([[w1], [w2]])
        return eta + (J @ xi) * self.dt, xi

    # === Timer principal ===
    def timer_callback(self):
        # Actualizar poses
        self.eta, self.xi = self.compute_pose(self.w1, self.w2, self.eta)
        self.eta_expected, self.xi_expected = self.compute_pose(self.w1_expected, self.w2_expected, self.eta_expected)

        # Publicar odometría y TF
        self.publish_odometry(self.eta, self.xi, "real_base_link", self.odom_real_pub)
        self.publish_tf(self.eta, "real_base_link")

        self.publish_odometry(self.eta_expected, self.xi_expected, "ideal_base_link", self.odom_ideal_pub)
        self.publish_tf(self.eta_expected, "ideal_base_link")

        # Publicar joint states
        self.publish_joint_states()

        # Publicar error
        self.publish_pose_error()

        self.get_logger().info(
            f"Real: x={self.eta[0,0]:.2f}, y={self.eta[1,0]:.2f} | "
            f"Ideal: x={self.eta_expected[0,0]:.2f}, y={self.eta_expected[1,0]:.2f}"
        )

    # === Odometría ===
    def publish_odometry(self, eta, xi, frame: str, publisher):
        now = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = frame

        odom_msg.pose.pose.position.x = eta[0, 0]
        odom_msg.pose.pose.position.y = eta[1, 0]
        odom_msg.pose.pose.orientation.z = np.sin(eta[2, 0] / 2)
        odom_msg.pose.pose.orientation.w = np.cos(eta[2, 0] / 2)

        odom_msg.twist.twist.linear.x = xi[0, 0]
        odom_msg.twist.twist.linear.y = xi[1, 0]
        odom_msg.twist.twist.angular.z = xi[2, 0]

        publisher.publish(odom_msg)

    # === TF ===
    def publish_tf(self, eta, frame: str):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = frame
        t.transform.translation.x = eta[0, 0]
        t.transform.translation.y = eta[1, 0]
        t.transform.rotation.z = np.sin(eta[2, 0] / 2)
        t.transform.rotation.w = np.cos(eta[2, 0] / 2)
        self.tf_broadcaster.sendTransform(t)

    # === Joint States ===
    def publish_joint_states(self):
        now = self.get_clock().now()

        self.th1_real += self.w1 * self.dt
        self.th2_real += self.w2 * self.dt
        self.th1_ideal += self.w1_expected * self.dt
        self.th2_ideal += self.w2_expected * self.dt

        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = [
            "real_left_wheel_joint", "real_right_wheel_joint",
            "ideal_left_wheel_joint", "ideal_right_wheel_joint"
        ]
        joint_msg.position = [
            self.th1_real, self.th2_real,
            self.th1_ideal, self.th2_ideal
        ]
        joint_msg.velocity = [
            self.w1, self.w2,
            self.w1_expected, self.w2_expected
        ]
        joint_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self.joint_state_pub.publish(joint_msg)

    # === Error ===
    def publish_pose_error(self):
        error_x = self.eta_expected[0, 0] - self.eta[0, 0]
        error_y = self.eta_expected[1, 0] - self.eta[1, 0]
        error_theta = (self.eta_expected[2, 0] - self.eta[2, 0] + np.pi) % (2 * np.pi) - np.pi

        msg = Float32MultiArray()
        msg.data = [error_x, error_y, np.degrees(error_theta)]
        self.error_pub.publish(msg)

        self.get_logger().info(
            f"Error -> Δx: {error_x:.3f}, Δy: {error_y:.3f}, Δθ: {np.degrees(error_theta):.2f}°"
        )

def main():
    rclpy.init()
    node = Direct_Kinematics_Diff_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
