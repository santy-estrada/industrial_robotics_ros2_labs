import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math
import numpy as np


class ScaraInverseKinematics(Node):
    def __init__(self):
        super().__init__('scara_inverse_kinematics')
        # ---- Parameters (declare + get) ----
        self.declare_parameter('publish_rate', 60.0)      # Hz

        self.rate_hz = float(self.get_parameter('publish_rate').value)
        
        # ---- State ----
        #Parameters of the robot
        self.r1 = 0.16  # Length of first arm (m)
        self.r2 = 0.143  # Length of second arm (m)
        self.delta3 = 0.0  # Minimum extension of prismatic joint (m)
        self.flag = False 

        #Desired position
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        
        #Output joint configuration
        self.pj1 = 0.0
        self.pj2 = 0.0
        self.pj3 = 0.0
        
        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'desired_pos', self._scara_configuration, qos)
        self.publisher_conf = self.create_publisher(Twist, 'inv_kin', qos)
        
        # ---- Timer ----
        self.last_time_conf = None
        self.dt_conf = 1.0/self.rate_hz
        self.timer_conf = self.create_timer(self.dt_conf, self.publish_conf)
        
    def _scara_inverse_kinematics(self):
        # Inverse kinematics for a SCARA robot
        # px, py, pz are the desired end-effector positions
        # pj1, pj2, pj3 are the resulting joint variables (angles in rad, extension in m)
        
        # Joint 1 and 2 (revolute)
        D = round((self.px**2 + self.py**2 - self.r1**2 - self.r2**2) / (2 * self.r1 * self.r2),6)
        if abs(D) > 1.0:
            self.get_logger().warn(f'Position out of reach: D = {D}')
            return
        
        #Two options for theta2: elbow up and elbow down
        theta2_opt = [math.acos(D), -math.acos(D)]
        theta1_opt = []

        for theta2 in theta2_opt:
            th1 = None
            if abs(math.degrees(theta2)) > 120:
                self.get_logger().warn(f'Position out of reach: |theta2| > 120 deg ({math.degrees(theta2):.2f} deg)')
                theta1_opt.append(None)
                continue

            mat = np.array([[self.r1 + self.r2 * math.cos(theta2), -self.r2 * math.sin(theta2)],
                            [self.r2 * math.sin(theta2), self.r1 + self.r2 * math.cos(theta2)]])
            vec = np.array([self.px, self.py])

            if np.linalg.det(mat) == 0:
                self.get_logger().warn('Singular configuration')
                theta1_opt.append(None)
                continue

            mat_inv = np.linalg.inv(mat)
            sol = mat_inv @ vec
            th1 = math.atan2(sol[1], sol[0])

            if abs(math.degrees(th1)) >= 80:
                self.get_logger().warn(f'Position out of reach: |theta1| > 80 deg ({math.degrees(th1):.2f} deg)')
                th1 = None

            theta1_opt.append(th1)

        # Select the first valid (th1, th2) pair
        found_valid = False
        prismatic = self.delta3 - self.pz
        
        for idx, th1 in enumerate(theta1_opt):
            th2 = theta2_opt[idx]
            if th1 is not None and th2 is not None and abs(prismatic) <= 14.0:
                self.pj1 = th1
                self.pj2 = th2
                self.pj3 = prismatic  # Extension of prismatic joint (m)
                found_valid = True
                break

        if not found_valid:
            self.get_logger().warn('No valid solution for joint angles; keeping previous values.')

            
    def _scara_configuration(self, msg: Twist):
        # The message sends angular velocity in deg/s. We convert it to rad/s
        # Angular position in deg, we convert it to rad
        # Linear position in mm, we convert it to m
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z

        self.px = x / 1000.0 # End effector  position (m)
        self.py = y / 1000.0 # End effector  position (m)
        self.pz = z / 1000.0 # End effector  position (m)       
        
        if not self.flag:
            self.flag = True


    def publish_conf(self):
        now = self.get_clock().now()
        if self.last_time_conf is None:
            self.last_time_conf = now
            return

        dt = (now - self.last_time_conf).nanoseconds * 1e-9
        self.last_time_conf = now
        if dt <= 0.0:
            return

        if self.flag:
            self._scara_inverse_kinematics()
        
            tw = Twist()
            tw.linear.x = math.degrees(self.pj1)
            tw.linear.y = math.degrees(self.pj2)
            tw.linear.z = self.pj3 * 1000.0  # Convert to mm
            self.publisher_conf.publish(tw)




def main(args=None):
    rclpy.init(args=args)
    node = ScaraInverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
