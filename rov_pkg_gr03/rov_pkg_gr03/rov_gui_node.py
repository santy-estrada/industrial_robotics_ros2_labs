#!/usr/bin/env python3
import sys
import rclpy
import signal  # <--- CAMBIO: Importamos el módulo de señales
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QProgressBar
from PySide6.QtCore import QTimer, Qt

# --- La clase RovGui (SIN CAMBIOS) ---
class RovGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROV Dashboard  submarine')
        
        # 1. Define tus widgets
        self.layout = QVBoxLayout()
        self.label_humedad = QLabel('Humedad: -- %')
        self.label_nivel = QLabel('Nivel Ballast:')
        self.bar_nivel = QProgressBar()
        self.label_finales_carrera = QLabel('Finales de Carrera: INDEFINIDO')
        
        # 2. Añádelos al layout
        self.layout.addWidget(self.label_humedad)
        self.layout.addWidget(self.label_nivel)
        self.layout.addWidget(self.bar_nivel)
        self.layout.addWidget(self.label_finales_carrera)
        self.setLayout(self.layout)

    # 3. Funciones para actualizar los datos desde ROS
    def actualizar_humedad(self, valor):
        self.label_humedad.setText(f'Humedad: {valor:.1f} %')
        if valor > 20.0: # Umbral de ejemplo
            self.label_humedad.setStyleSheet('color: red; font-weight: bold;')
        else:
            self.label_humedad.setStyleSheet('color: black;')
            
    def actualizar_nivel(self, valor):
        self.bar_nivel.setValue(int(valor))
        
    def actualizar_finales_carrera(self, lleno, vacio):
        estado_lleno = lleno > 0.5 
        estado_vacio = vacio > 0.5

        if estado_lleno:
            self.label_finales_carrera.setText('Finales de Carrera: LLENO 🟢')
            self.label_finales_carrera.setStyleSheet('color: green;')
        elif estado_vacio:
            self.label_finales_carrera.setText('Finales de Carrera: VACÍO 🔵')
            self.label_finales_carrera.setStyleSheet('color: blue;')
        else:
            self.label_finales_carrera.setText('Finales de Carrera: En movimiento...')
            self.label_finales_carrera.setStyleSheet('color: gray;')

# --- La clase GuiNode (SIN CAMBIOS) ---
class GuiNode(Node):
    def __init__(self, gui_app):
        super().__init__('rov_gui_node')
        self.gui = gui_app 

        self.create_subscription(
            Float32MultiArray,
            '/rov_status',
            self.status_callback,
            10)

    def status_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('Mensaje de /rov_status incompleto. Se esperan 4 elementos.', once=True)
            return

        humedad = msg.data[0]
        nivel_ballast = msg.data[1]
        final_carrera_lleno = msg.data[2]
        final_carrera_vacio = msg.data[3]

        self.gui.actualizar_humedad(humedad)
        self.gui.actualizar_nivel(nivel_ballast)
        self.gui.actualizar_finales_carrera(
            final_carrera_lleno,
            final_carrera_vacio
        )


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    gui_window = RovGui()
    ros_node = GuiNode(gui_app=gui_window)
    
    # El "Truco del Timer" (aquí guardamos la referencia al timer)
    timer = QTimer() # <--- CAMBIO (Guardamos el timer en una variable)
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(50) 

    # --- CAMBIO: Definimos el manejador de Ctrl+C ---
    def shutdown_handler(sig, frame):
        """
        Manejador para Ctrl+C (SIGINT).
        Esto se ejecutará en lugar de lanzar KeyboardInterrupt.
        """
        print() # Imprime una nueva línea después del ^C
        ros_node.get_logger().info('Cerrando nodo (Ctrl+C)...')
        
        # 1. Detener el timer que llama a spin_once
        timer.stop()
        
        # 2. Detener la aplicación de la GUI
        app.quit() # Esto hará que app.exec() termine y devuelva el control
        
        # 3. Destruir el nodo de ROS
        ros_node.destroy_node()

    # Conectamos la señal SIGINT (Ctrl+C) a nuestro manejador
    signal.signal(signal.SIGINT, shutdown_handler)
    # --- FIN DEL CAMBIO ---

    gui_window.show()
    
    # --- CAMBIO: Modificamos cómo se llama y se cierra ---
    # app.exec() bloquea hasta que app.quit() es llamado
    exit_code = app.exec()
    
    # Cuando el loop de la GUI termina (gracias a app.quit()),
    # ahora SÍ podemos llamar a rclpy.shutdown()
    rclpy.shutdown()
    sys.exit(exit_code)
    # --- FIN DEL CAMBIO ---

if __name__ == '__main__':
    main()