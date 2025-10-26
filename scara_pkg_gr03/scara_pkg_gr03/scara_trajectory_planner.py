import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import json
from std_msgs.msg import String
class ScaraTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('scara_trajectory_planner')

       # Subs 
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.create_subscription(PointCloud, 'dxf_pointcloud', self.pointcloud_callback, qos_profile=qos)
        self.create_subscription(String, 'dxf_figures_info', self.figures_info_callback, qos_profile=qos)
        # Params
        self.declare_parameter('publish_rate', 60.0)  # Hz
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # velocidades para cada tipo de trayectoria
        self.polyline_time = 3   
        self.circle_time = 0.2     
        self.transition_time = 1.0
        #flag de transición
        self.in_transition = False

        #  Pub
        self.publisher = self.create_publisher(Twist, 'desired_pos', 10)

        # tiempo entre publicaciones
        self.dt = 1.0 / self.publish_rate
        self.timer = None

        # Variables
        self.all_points = [] #contiene todos lo spuntos del Pointcloud que llega
        self.figures_info = [] #contiene información de las figuras detectadas y la cantidad de puntos secuenciales que tiene cada una
        self.current_figure_index = 0  #indica de manera incremental la figura que se está graficando
        self.current_path = [] #subespacio de puntos totales, almacena puntos de trayectoria actual y cierra figura
        self.segment_index = 0 #indica el segmento ( union entre dos puntos consecutivos) en el que se está interpolando, esto se mide respecto a current path
        self.steps = 0 #almacena el total de pasos de interpolación del segmento actual, sirve para comparar con segment_index
        self.current_step = 0 # almacena el paso de interpolación que se está realizando en el segmento, aquí se hace el update 
        self.current_point = None #guarda el estado de interpolación para actualizar el publisher
        self.delta = None #es el vector de despllazamiento entre pasos.

    def pointcloud_callback(self, msg: PointCloud): #recibe información del topic pointcloud y genera array (nx3) con todos los puntos de las trayectorias a realizar
        self.all_points = np.array([[pt.x, pt.y, pt.z] for pt in msg.points])
        self.get_logger().info(f"Recibidos {len(self.all_points)} puntos en dxf_pointcloud.")

        # Si ya tenemos figures_info ( ya sabemos que puntos de all points pertenecen a que trayectoria), podemos armar trayectorias
        if self.figures_info:
            self.start_next_figure()

    def figures_info_callback(self, msg): #esta función lee e importa la información de orden del mensaje pointcloud
        try:
            self.figures_info = json.loads(msg.data) #convieret el mensaje string json en objeto de python
            self.get_logger().info(f"Recibida info de figuras: {self.figures_info}")
        except Exception as e:
            self.get_logger().error(f"Error parseando JSON en dxf_figures_info: {e}")
            return

        if len(self.all_points) > 0: #garantiza que ya llegó la información de los puntos a graficar
            self.start_next_figure()

    def start_next_figure(self):
        if self.current_figure_index >= len(self.figures_info): #garantiza que todavia hay figuras por graficar
            self.get_logger().info("Todas las figuras completadas.")
            if self.timer:
                self.timer.cancel()
            return

        fig = self.figures_info[self.current_figure_index]  #almacena la información de la proxima figura a graficar
        num_points = fig["num_points"]
        fig_type = fig["type"]

        # Extraer puntos de esta figura
        start_idx = sum(f["num_points"] for f in self.figures_info[:self.current_figure_index]) #busca el punto incial de la proxima figura a graficar en función de la canidad de puntos de las figuras anteriores
        end_idx = start_idx + num_points #genera el indice de fin de figura sumando la cantidad de puntos de la figura según figure info
        self.current_path = self.all_points[start_idx:end_idx] # llena array current path con puntos de all point que pertenen a la figura actual

  
        if len(self.current_path) > 1:  #agrega primer punto como ultimo para cerrar figura
            self.current_path = np.vstack([self.current_path, self.current_path[0]])

        # Definir tiempo entre puntos según tipo
        if "CIRCLE" in fig_type:
            self.move_time = self.circle_time
        else:
            self.move_time = self.polyline_time

        self.get_logger().info(f"Iniciando figura {self.current_figure_index+1}/{len(self.figures_info)} "
                            f"({fig_type}, {num_points} puntos) con move_time={self.move_time}")

        # Resetear interpolador
        self.segment_index = 0
        self.start_segment()

        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(self.dt, self.update)

    def start_segment(self):
        
        if self.segment_index < len(self.current_path) - 1: # comprueba que se tengan segmentos por interpolar
            p1 = self.current_path[self.segment_index] # asigna waypoints de interpolación
            p2 = self.current_path[self.segment_index + 1]

            self.steps = int(self.move_time * self.publish_rate) #define cantidad de puntos de interpolación en función de tiempo
            self.current_step = 0
            self.current_point = p1.copy() #comienza en p1 y se actualiza con el update
            self.delta = (p2 - p1) / self.steps # define el cambio de interpolación para cada step

            self.get_logger().info(f"Interpolando de {p1} a {p2} en {self.move_time}s.")
            return

        
        self.get_logger().info("Figura completada.")

        last_point = self.current_path[-1]
        self.current_figure_index += 1

        if self.current_figure_index < len(self.figures_info):
            # Hay otra figura → transición al primer punto de la siguiente
            next_fig = self.figures_info[self.current_figure_index]
            start_idx = sum(f["num_points"] for f in self.figures_info[:self.current_figure_index]) #busca punto inicial de la siguiente figura
            next_first_point = self.all_points[start_idx]

            self.steps = int(self.transition_time * self.publish_rate) # define tiempo de transición para proxima figura
            self.current_step = 0
            self.current_point = last_point.copy()
            self.delta = (next_first_point - last_point) / self.steps
            self.in_transition = True #genera flag para identifcar la transición

            self.get_logger().info(f"Transición de {last_point} → {next_first_point} "
                                   f"en {self.transition_time}s.")
        else:
            # No hay más figuras
            self.get_logger().info("Todas las figuras completadas.")
            if self.timer:
                self.timer.cancel()

    def update(self):
        if self.current_step <= self.steps: #todavia quedan puntos en el segmento de itnerpolación
            tw = Twist()
            tw.linear.x = float(self.current_point[0])
            tw.linear.y = float(self.current_point[1])

            if not self.in_transition:
                # Durante toda la figura: z = 15
                tw.linear.z = -0.0
            else:
                # En transiciones: sube la herramienta
                tw.linear.z = 10.0

            self.publisher.publish(tw) #publica pose objetivo de la herramienta

            # avanzar interpolación
            self.current_point = self.current_point + self.delta #avanza un paso de interpolación
            self.current_step += 1
        else:
            if self.in_transition:
                # transición terminada → iniciar siguiente figura
                self.in_transition = False #quita fflag
                self.start_next_figure()
            else:
                # si termina el segmento, que siga con el proximo
                self.segment_index += 1

                if self.segment_index >= len(self.current_path) - 1: #si ya no hay segmentos, se acabó la figura
                    
                    tw = Twist()
                    tw.linear.x = float(self.current_point[0])
                    tw.linear.y = float(self.current_point[1])
                    tw.linear.z = 0.0
                    self.publisher.publish(tw)

                self.start_segment()

def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
