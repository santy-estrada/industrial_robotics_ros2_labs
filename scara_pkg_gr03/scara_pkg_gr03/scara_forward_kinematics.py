import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile
import math
import numpy as np


class ScaraJointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        # ---- Parameters (declare + get) ----
        self.declare_parameter('joint1', 'base_to_arm1_joint')
        self.declare_parameter('joint2', 'arm_1_arm_2_joint')
        self.declare_parameter('joint3', 'arm_2_prismatic_joint')

        self.declare_parameter('publish_rate', 20.0)      # Hz
        self.declare_parameter('max_path_size', 10000000)  # Max number of poses in path buffer
        self.declare_parameter('min_distance_m', 0.0001)  # Minimum distance (0.1mm) to publish new pose

        self.joint1 = self.get_parameter('joint1').get_parameter_value().string_value
        self.joint2 = self.get_parameter('joint2').get_parameter_value().string_value
        self.joint3 = self.get_parameter('joint3').get_parameter_value().string_value

        self.rate_hz = float(self.get_parameter('publish_rate').value)
        self.max_path_size = int(self.get_parameter('max_path_size').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        
        # ---- State ----
        self.pj1 = 0.0
        self.vj1 = 0.0 
        self.pj2 = 0.0
        self.vj2 = 0.0
        self.pj3 = 0.0
        
        # ---- Transformations ----
        self.endEffector_x = 0.0
        self.endEffector_y = 0.0
        self.endEffector_z = 0.0
        
        # Path tracking with distance filter
        self.last_published_path_point = None  # (x, y, z) of last published point
                                                                      #0.143 or 0.155
        self.dh_matrix = np.array([[0, 0, 0.082, 0], [0.16, 0, 0, 0], [0.155, math.pi, 0.018, 0], [0, 0, 0.064, 0]], dtype=float) # [a, alpha, Sj, thetaj]    

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/scara_measurements', self._scara_configuration, qos)      #Connect to scara_conf if there is no pico; else scara_measurements
        self.publisher_joint_states = self.create_publisher(JointState, 'joint_states', qos)
        self.publisher_end_effector = self.create_publisher(Twist, 'end_effector', qos)
        
        # Publisher for actual path visualization in RViz
        self.path_publisher = self.create_publisher(Path, 'end_effector_path', qos)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'base_link'
        
        # ---- Timer ----
        self.last_time_joint_states = None
        self.dt_joint_states = 1.0 / self.rate_hz
        self.timer_joint_states = self.create_timer(self.dt_joint_states, self.publish_joint_states)

        self.last_time_end_effector = None
        self.dt_end_effector = 1.0 / self.rate_hz  # Use same rate as joint states
        self.timer_end_effector = self.create_timer(self.dt_end_effector, self.publish_end_effector)
        self.end_effector = Twist()


    def _T_DH(self, a_ij, alpha_ij, s_i, theta_i):
        cos_theta = math.cos(theta_i)
        sin_theta = math.sin(theta_i)
        cos_alpha = math.cos(alpha_ij)
        sin_alpha = math.sin(alpha_ij)
        T_ij = np.array([
            [cos_theta, -sin_theta, 0, a_ij],
            [cos_alpha * sin_theta, cos_alpha * cos_theta, -sin_alpha, -s_i * sin_alpha],
            [sin_alpha * sin_theta, sin_alpha * cos_theta, cos_alpha, s_i * cos_alpha],
            [0, 0, 0, 1]
        ], dtype=float)
        return T_ij
    

    def _scara_forward_kinematics(self):
        # Update DH matrix with new joint values
        self.dh_matrix[0][3] = self.pj1
        self.dh_matrix[1][3] = self.pj2
        self.dh_matrix[3][2] = self.pj3

        DoF = self.dh_matrix.shape[0]
        T_ij = np.zeros((4, 4, DoF))
        for j in range(DoF):
            T_ij[:, :, j] = self._T_DH(self.dh_matrix[j, 0], self.dh_matrix[j, 1], self.dh_matrix[j, 2], self.dh_matrix[j, 3])

        # Cumulative transformation matrices
        T = np.zeros((4, 4, DoF))
        T[:, :, 0] = T_ij[:, :, 0]
        for i in range(1, DoF):
            T[:, :, i] = T[:, :, i-1] @ T_ij[:, :, i]

        # End-effector pose is T[:, :, -1]
        end_effector_pose = T[:, :, -1]
        
        self.endEffector_x = end_effector_pose[0, 3]
        self.endEffector_y = end_effector_pose[1, 3]
        self.endEffector_z = end_effector_pose[2, 3]

        # Log computed positions for each joint and end effector
        # Commented out loop for all joints
        # for idx, label in enumerate(["Joint 1", "Joint 2", "Joint 3", "End Effector"]):
        #     pos = T[:, 3, idx]  # last column, first 3 rows
        #     self.get_logger().info(f"{label} position: x={pos[0]:.4f} m, y={pos[1]:.4f} m, z={pos[2]:.4f} m")
            
        # Only log end effector position
        pos = T[:, 3, -1]  # End effector is the last transformation
        self.get_logger().info(f"End Effector position: x={pos[0]:.4f} m, y={pos[1]:.4f} m, z={pos[2]:.4f} m")
            
    def _scara_configuration(self, msg: Twist):
        # The message sends angular velocity in deg/s. We convert it to rad/s
        # Angular position in deg, we convert it to rad
        # Linear position in mm, we convert it to m
        self.pj1 = math.radians(msg.linear.x)  # Joint 1 position (rad)
        self.pj2 = math.radians(msg.linear.y)  # Joint 2 position (rad)
        
        # Prismatic joint (pj3) conversion:
        # msg.linear.z is in mm, where:
        #   0 mm  → end effector at ground level (z = 0)
        #   -10 mm → end effector 10mm above ground (z = 0.01m)
        #   -5 mm  → end effector 5mm below ground (z = -0.005m)
        # Formula: end_effector_z = -msg.linear.z / 1000.0
        # pj3 = end_effector_z + DH_offset (0.064m)
        self.pj3 = (msg.linear.z / 1000.0) + 0.064
        
        self.vj1 = math.radians(msg.angular.x) # Joint 1 velocity (rad/s)
        self.vj2 = math.radians(msg.angular.y) # Joint 2 velocity (rad/s)
        # Joint 3 velocity is assumed always 0 (prismatic joint)       
        
    def publish_end_effector(self):
        now = self.get_clock().now()
        if self.last_time_end_effector is None:
            self.last_time_end_effector = now
            return

        dt = (now - self.last_time_end_effector).nanoseconds * 1e-9
        self.last_time_end_effector = now
        if dt <= 0.0:
            return

        self._scara_forward_kinematics()
        
        tw = Twist()
        tw.linear.x = self.endEffector_x
        tw.linear.y = self.endEffector_y
        tw.linear.z = self.endEffector_z
        self.publisher_end_effector.publish(tw)

        # Apply 0.1mm distance filter before adding to path
        should_publish = False
        if self.last_published_path_point is None:
            # Always publish first point
            should_publish = True
        else:
            # Calculate distance from last published point
            dx = self.endEffector_x - self.last_published_path_point[0]
            dy = self.endEffector_y - self.last_published_path_point[1]
            dz = self.endEffector_z - self.last_published_path_point[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Only publish if distance >= 0.1mm (0.0001m)
            if distance >= self.min_distance_m:
                should_publish = True
        
        if should_publish:
            # Create PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = now.to_msg()
            pose.header.frame_id = self.path_msg.header.frame_id
            
            # Fill position
            pose.pose.position.x = self.endEffector_x
            pose.pose.position.y = -self.endEffector_y
            pose.pose.position.z = self.endEffector_z
            pose.pose.orientation.w = 1.0

            # Add new pose to path
            self.path_msg.poses.append(pose)

            # Implement buffer size limit
            if len(self.path_msg.poses) > self.max_path_size:
                self.path_msg.poses.pop(0)

            # Update last published point
            self.last_published_path_point = (self.endEffector_x, self.endEffector_y, self.endEffector_z)

            # Update path header timestamp and publish
            self.path_msg.header.stamp = now.to_msg()
            self.path_publisher.publish(self.path_msg)


    def publish_joint_states(self):
        now = self.get_clock().now()
        if self.last_time_joint_states is None:
            self.last_time_joint_states = now
            return

        dt = (now - self.last_time_joint_states).nanoseconds * 1e-9
        self.last_time_joint_states = now
        if dt <= 0.0:
            return

        # Publish joint_states 
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.joint1, self.joint2, self.joint3]
        js.position = [-self.pj1, -self.pj2, self.pj3]
        js.velocity = [-self.vj1, -self.vj2, 0.0]
        self.publisher_joint_states.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ScaraJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
