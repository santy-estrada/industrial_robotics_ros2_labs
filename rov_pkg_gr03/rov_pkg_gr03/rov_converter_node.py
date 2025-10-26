#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy 

# --- CONFIGURACIÓN DE BOTONES JOYSTICK ---
# ¡Verifica tus índices con `ros2 topic echo /joy`!
BTN_A = 0    # Botón A
BTN_B = 1    # Botón B
BTN_LB = 4   # Botón Left Bumper (LB) <--- NUEVO
# ----------------------------------------

# Límite de potencia (1.0 = -1.0 a 1.0)
MAX_PERCENTAGE = 1.0 
# Frecuencia de publicación (10 Hz = cada 0.1s)
PUB_FREQUENCY = 10.0 

class RovConverterNode(Node):
    def __init__(self):
        super().__init__('rov_converter_node')
        
        # --- Suscriptor 1 (Ejes procesados) ---
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel_joy',  
            self.twist_callback, 
            10)
        
        # --- Suscriptor 2 (Botones crudos) ---
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback, 
            10)
        
        # Publicador (sin cambios)
        self.motor_pub = self.create_publisher(
            Twist,
            'rov_motor_control', 
            10)
            
        self.get_logger().info('Nodo ROV Converter (2 subs) iniciado.')
        # <--- NUEVO (Log actualizado)
        self.get_logger().info('Salida: Twist -> [lx,ly,lz,ax,ay,az] = [Izq, Der, Ballast, A, B, LB]')
        
        # --- Variables de estado (para sincronizar) ---
        self.surge_input = 0.0
        self.heave_input = 0.0
        self.yaw_input = 0.0
        self.btn_a_input = 0.0
        self.btn_b_input = 0.0
        self.btn_lb_input = 0.0  # <--- NUEVO
        
        # Mensaje de salida
        self.motor_command_msg = Twist()

        # Timer para publicar
        self.timer = self.create_timer(1.0 / PUB_FREQUENCY, self.timer_callback)

    def twist_callback(self, msg: Twist):
        """Callback de /cmd_vel_joy. SOLO guarda el estado de los ejes."""
        self.surge_input = msg.linear.x
        self.heave_input = msg.linear.z
        self.yaw_input = msg.angular.z

    def joy_callback(self, msg: Joy):
        """Callback de /joy. SOLO guarda el estado de los botones."""
        try:
            self.btn_a_input = float(msg.buttons[BTN_A])
            self.btn_b_input = float(msg.buttons[BTN_B])
            self.btn_lb_input = float(msg.buttons[BTN_LB]) # <--- NUEVO
        except IndexError:
            self.get_logger().warn('Índices de botones no encontrados. Revisa la configuración.', once=True)

    def timer_callback(self):
        """Publica el estado conocido 10 veces por segundo."""
        
        # --- 1. Lógica de Mezcla ---
        motor_left_raw = self.surge_input - self.yaw_input
        motor_right_raw = self.surge_input + self.yaw_input
        motor_ballast_raw = self.heave_input

        # --- 2. Recorte ---
        motor_left = np.clip(motor_left_raw, -1.0, 1.0)
        motor_right = np.clip(motor_right_raw, -1.0, 1.0)
        motor_ballast = np.clip(motor_ballast_raw, -1.0, 1.0)
        
        # --- 3. Escalar ---
        motor_left_cmd = motor_left * MAX_PERCENTAGE
        motor_right_cmd = motor_right * MAX_PERCENTAGE
        motor_ballast_cmd = motor_ballast * MAX_PERCENTAGE
        
        # --- 4. Llenar el mensaje Twist de salida ---
        self.motor_command_msg.linear.y = motor_left_cmd
        self.motor_command_msg.linear.x = motor_right_cmd
        self.motor_command_msg.linear.z = motor_ballast_cmd
        
        self.motor_command_msg.angular.x = self.btn_a_input
        self.motor_command_msg.angular.y = self.btn_b_input
        self.motor_command_msg.angular.z = self.btn_lb_input # <--- NUEVO
        
        # --- 5. Publicar el comando ---
        self.motor_pub.publish(self.motor_command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RovConverterNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()