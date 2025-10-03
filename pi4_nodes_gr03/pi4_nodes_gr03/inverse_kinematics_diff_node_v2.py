#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class InverseKinematicsDiff(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Parámetros del robot
        self.R = 0.042 / 2                      # Radio de rueda (m)
        self.b = 0.12766 / 2 + 0.019 / 2        # Mitad de la distancia entre ruedas (m)
        self.max_rpm = 130.0                    # Límite de velocidad en RPM

        # Velocidades del comando
        self.u = 0.0  # Velocidad lineal (m/s)
        self.r = 0.0  # Velocidad angular (rad/s)
        
        # Timeout control
        self.cmd_vel_timeout = 5.0  # 5 seconds timeout
        self.last_cmd_vel_time = None
        self.should_publish = False

        # Suscriptor al comando de velocidad deseado
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 🆕 Publicador de velocidades deseadas de las ruedas en formato Twist
        self.wheel_pub = self.create_publisher(Twist, 'wheel_setpoint', 10)

        # Timer para cálculo periódico
        self.dt = 0.1
        self.create_timer(self.dt, self.timer_callback)
        
        self.twist_msg = Twist()

    def cmd_vel_callback(self, msg: Twist):
        self.u = msg.linear.x
        self.r = msg.angular.z
        self.last_cmd_vel_time = time.time()
        self.should_publish = True
        self.get_logger().info(f'[CMD] u={self.u:.2f} m/s | r={self.r:.2f} rad/s')

    def timer_callback(self):
        # Check if we should stop publishing due to timeout
        if self.last_cmd_vel_time is not None:
            time_since_last_cmd = time.time() - self.last_cmd_vel_time
            if time_since_last_cmd > self.cmd_vel_timeout:
                if self.should_publish:
                    self.get_logger().info('[TIMEOUT] No cmd_vel received in 5s, stopping publication')
                    self.should_publish = False
                return
        elif self.last_cmd_vel_time is None:
            # No cmd_vel received yet
            return
            
        # Only publish if we should
        if not self.should_publish:
            return

        # Cinemática inversa (rad/s)
        w1_rad = (self.u - (self.r * self.b)) / self.R  # Rueda izquierda
        w2_rad = (self.u + (self.r * self.b)) / self.R  # Rueda derecha

        # Convertir a RPM
        w1_rpm = self.rad_s_to_rpm(w1_rad)
        w2_rpm = self.rad_s_to_rpm(w2_rad)

        # Limitar a velocidad máxima
        w1_rpm = max(min(w1_rpm, self.max_rpm), -self.max_rpm)
        w2_rpm = max(min(w2_rpm, self.max_rpm), -self.max_rpm)

        # 🆕 Publicar como Twist
        self.twist_msg.linear.x = w1_rpm
        self.twist_msg.angular.z = w2_rpm
        self.wheel_pub.publish(self.twist_msg)

        self.get_logger().info(f'[SETPOINT] w1={w1_rpm:.2f} RPM | w2={w2_rpm:.2f} RPM')

    def rad_s_to_rpm(self, rad_s):
        return rad_s * 60 / (2 * 3.1416)

def main():
    rclpy.init()
    node = InverseKinematicsDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
