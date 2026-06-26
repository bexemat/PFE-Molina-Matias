#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Suscripción al tópico de la GUI
        self.subscription = self.create_subscription(
            String, 'arm_commands', self.gui_callback, 10)
        
        # Configuración del puerto serie (Ajustar /dev/ttyACM0 según tu placa Nucleo)
        # El baudrate estándar de 115200 es excelente para telemetría robótica
        self.puerto_serie = '/dev/ttyACM0' 
        self.baudrate = 115200
        
        try:
            self.stm32 = serial.Serial(self.puerto_serie, self.baudrate, timeout=0.1)
            time.sleep(2) # Espera de cortesía para la estabilización del micro
            self.get_logger().info(f'Conexión serie exitosa en: {self.puerto_serie}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir el puerto serie: {e}')
            self.get_logger().warn('El nodo correrá en modo SIMULACIÓN (sin hardware).')
            self.stm32 = None

    def gui_callback(self, msg):
        """
        Recibe el string de la GUI: 'CMD_ANGULOS: q1=90, q2=45, q3=0, SOL=0'
        Y lo transforma en una trama compacta para la STM32: '<90,45,0,0>\n'
        """
        try:
            # Parseo básico del mensaje de la GUI
            datos = msg.data.replace("", "").split(":")[-1].strip()
            partes = datos.split(",")
            
            q1 = partes[0].split("=")[-1]
            q2 = partes[1].split("=")[-1]
            q3 = partes[2].split("=")[-1]
            sol = partes[3].split("=")[-1]
            
            # Formato empaquetado industrial: <q1,q2,q3,sol>\n
            trama_compacta = f"<{q1},{q2},{q3},{sol}>\n"
            
            if self.stm32 and self.stm32.is_open:
                self.stm32.write(trama_compacta.encode('utf-8'))
                self.get_logger().info(f'Enviado a STM32: {trama_compacta.strip()}')
            else:
                self.get_logger().info(f'[Simulación Serie]: {trama_compacta.strip()}')
                
        except Exception as e:
            self.get_logger().error(f'Error al procesar o enviar la trama: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Apagando puente serie...')
    finally:
        if node.stm32 and node.stm32.is_open:
            node.stm32.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()