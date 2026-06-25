#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ArmControllerNode(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre 'arm_controller'
        super().__init__('arm_controller')
        
        # Log para verificar que el nodo arrancó
        self.get_logger().info('Nodo Arm Controller inicializado correctamente.')
        
        # Creamos un temporizador que ejecuta la función cada 1.0 segundos
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.contador = 0

    def timer_callback(self):
        self.contador += 1
        self.get_logger().info(f'Sincronizando estado del brazo... Ciclo: {self.contador}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Apagando nodo Arm Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()