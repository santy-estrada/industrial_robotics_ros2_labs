#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# --- CONFIGURACIÓN DE TU JOYSTICK ---
# ¡¡MUY IMPORTANTE!! 
# Verifica tus índices con `ros2 topic echo /joy`
AXIS_SURGE = 1   # Palanca Izq Y (Avance)
AXIS_YAW = 0     # Palanca Izq X (Giro)
AXIS_HEAVE = 3  # Palanca Der Y (Ballast)

BTN_A = 0        # Botón A
BTN_B = 1        # Botón B
BTN_LB = 4       # Botón LB (Enviado en angular.z)
BTN_RB = 5       # Botón RB (Nuestro "Hombre Muerto") <--- CAMBIO

# ----------------------------------------

# Límite de potencia (1.0 = -1.0 a 1.0)
MAX_PERCENTAGE = 1.0 
# Frecuencia de publicación (10 Hz = cada 0.1s)
PUB_FREQUENCY = 10.0 

class RovTeleopNode(Node):
    def __init__(self):
        super().__init__('rov_teleop_node')
        
        # Suscriptor directo a /joy
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',  # El topic estándar de joystick
            self.joy_callback,
            10)
        
        # Publicador directo a la Pico
        self.motor_pub = self.create_publisher(
            Twist,
            'rov_motor_control', # Topic para la Pico
            10)
            
        self.get_logger().info('Nodo ROV Teleop (Directo) iniciado. ¡Sin delay!')
        self.get_logger().info('Hombre Muerto: RB (Botón 5)') # <--- CAMBIO (Log)
        self.get_logger().info('Salida: [lx,ly,lz,ax,ay,az] = [Izq, Der, Ballast, A, B, LB]')
        
        # --- Variables para guardar el estado del joystick ---
        self.surge_input = 0.0
        self.yaw_input = 0.0
        self.heave_input = 0.0
        self.btn_a_input = 0.0
        self.btn_b_input = 0.0
        self.btn_lb_input = 0.0
        self.btn_rb_input = 0.0 # <--- CAMBIO

        # Mensaje de salida
        self.motor_command_msg = Twist()
        
        # Timer para publicar a una frecuencia constante
        self.timer = self.create_timer(1.0 / PUB_FREQUENCY, self.timer_callback)

    def joy_callback(self, msg: Joy):
        """Este callback SÓLO actualiza las variables de estado."""
        try:
            # Leemos los ejes
            self.surge_input = msg.axes[AXIS_SURGE] 
            self.yaw_input = msg.axes[AXIS_YAW]
            self.heave_input = msg.axes[AXIS_HEAVE]
            
            # Leemos los botones
            self.btn_a_input = float(msg.buttons[BTN_A])
            self.btn_b_input = float(msg.buttons[BTN_B])
            self.btn_lb_input = float(msg.buttons[BTN_LB])
            self.btn_rb_input = float(msg.buttons[BTN_RB]) # <--- CAMBIO

        except IndexError:
            self.get_logger().warn('Índices de joystick no encontrados. Revisa la configuración.', once=True)


    def timer_callback(self):
        """
        Este callback SÍ publica.
        Se ejecuta 10 veces por segundo, independientemente del joystick.
        """
        
        # Inicializamos todos los comandos a 0.0
        motor_left_cmd = 0.0
        motor_right_cmd = 0.0
        motor_ballast_cmd = 0.0
        button_a_cmd = 0.0
        button_b_cmd = 0.0
        
        # Siempre enviamos el estado de LB, esté o no presionado RB
        button_lb_cmd = self.btn_lb_input # <--- CAMBIO (Lógica movida)

        # --- Lógica de "Hombre Muerto" (Dead-man's switch) ---
        # Si RB (nuestro botón de "hombre muerto") está presionado...
        if self.btn_rb_input > 0.5: # <--- CAMBIO (Variable)
            # 1. Lógica de Mezcla (Mixing)
            motor_left_raw = self.surge_input - self.yaw_input
            motor_right_raw = self.surge_input + self.yaw_input
            motor_ballast_raw = self.heave_input

            # 2. Recorte (Clipping)
            motor_left = np.clip(motor_left_raw, -1.0, 1.0)
            motor_right = np.clip(motor_right_raw, -1.0, 1.0)
            motor_ballast = np.clip(motor_ballast_raw, -1.0, 1.0)
            
            # 3. Escalar (aunque MAX_PERCENTAGE es 1.0)
            motor_left_cmd = motor_left * MAX_PERCENTAGE
            motor_right_cmd = motor_right * MAX_PERCENTAGE
            motor_ballast_cmd = motor_ballast * MAX_PERCENTAGE
            
            # 4. Pasar el estado de los botones (excepto LB, que ya se asignó)
            button_a_cmd = self.btn_a_input
            button_b_cmd = self.btn_b_input
        
        # Si RB NO está presionado, todos los valores (excepto LB) se quedan en 0.0
        
        # --- Publicar el mensaje Twist ---
        self.motor_command_msg.linear.y = -motor_left_cmd
        self.motor_command_msg.linear.x = -motor_right_cmd
        self.motor_command_msg.linear.z = motor_ballast_cmd
        
        self.motor_command_msg.angular.x = button_a_cmd
        self.motor_command_msg.angular.y = button_b_cmd
        self.motor_command_msg.angular.z = button_lb_cmd # <--- CAMBIO (Ahora envía LB)
        
        self.motor_pub.publish(self.motor_command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RovTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()