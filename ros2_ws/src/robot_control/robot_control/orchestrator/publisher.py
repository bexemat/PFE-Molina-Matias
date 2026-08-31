#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Point
from extra_interfaces.msg import Trama


class MinimalPublisher(Node):
    """
    Nodo Orquestador / Hardware Interface Bridge.
    Administra y rutea los comandos entre la GUI, el Planificador y micro-ROS.
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        self.is_estop_active = False

        # --- PUBLICADORES HACIA MICROROS (STM32 HARDWARE INTERFACE) ---
        self.micro_p2p_pub = self.create_publisher(Point, '/microROS/p2p_cmd', 10)
        self.micro_homing_pub = self.create_publisher(Bool, '/microROS/homing', 10)
        self.micro_magnet_pub = self.create_publisher(Bool, '/microROS/electroiman', 10)
        self.micro_estop_pub = self.create_publisher(Bool, '/microROS/emergency_stop', 10)

        # --- PUBLICADORES HACIA OTROS NODOS ROS 2 ---
        self.planner_estop_pub = self.create_publisher(Bool, '/planner/emergency_stop', 10)

        # --- SUSCRIPTORES DESDE LA INTERFAZ DE USUARIO (UI) ---
        self.ui_homing_sub = self.create_subscription(
            Bool, '/robot_ui/homing', self._ui_homing_callback, 10
        )
        self.ui_magnet_sub = self.create_subscription(
            Bool, '/robot_ui/e_magnet_on', self._ui_magnet_callback, 10
        )
        self.ui_estop_sub = self.create_subscription(
            Bool, '/robot_ui/emergency_stop', self._ui_estop_callback, 10
        )
        self.ui_p2p_sub = self.create_subscription(
            Point, '/robot_ui/p2p_cmd', self._ui_p2p_callback, 10
        )

        # --- SUSCRIPTORES DESDE EL PLANIFICADOR DE TRAYECTORIAS ---
        # Escucha el estado del planificador (1: Ejecutando, 2: Completado, -1: Error)
        self.planner_status_sub = self.create_subscription(
            Int8, '/planner/traj_status', self._planner_status_callback, 10
        )

        self.get_logger().info("Orquestador Central (Bridge Hardware) inicializado correctamente.")

    # -------------------------------------------------------------
    # CALLBACKS DE LA INTERFAZ DE USUARIO (UI)
    # -------------------------------------------------------------

    def _ui_p2p_callback(self, msg: Point):
        if self.is_estop_active:
            self.get_logger().warn("🚨 E-Stop activo: Consigna P2P descartada.")
            return

        self.get_logger().info(
            f"🚀 Consigna Joint P2P enviada a STM32 -> Q1={msg.x:.1f}°, Q2={msg.y:.1f}°, Q3={msg.z:.1f}°"
        )
        self.micro_p2p_pub.publish(msg)

    def _ui_homing_callback(self, msg: Bool):
        if msg.data and not self.is_estop_active:
            self.get_logger().info("🏠 Orden de Homing enviada a micro-ROS...")
            self.micro_homing_pub.publish(msg)

    def _ui_magnet_callback(self, msg: Bool):
        self.micro_magnet_pub.publish(msg)
        estado = "ENCENDIDO 🧲" if msg.data else "APAGADO"
        self.get_logger().info(f"🧲 Orden de Electroimán enviada: {estado}")

    def _ui_estop_callback(self, msg: Bool):
        self.is_estop_active = msg.data
        
        # Reenviar orden de E-Stop a micro-ROS y al planificador
        self.micro_estop_pub.publish(msg)
        self.planner_estop_pub.publish(msg)

        if self.is_estop_active:
            self.get_logger().error("🚨 PARADA DE EMERGENCIA ACTIVADA EN EL ORQUESTADOR.")
        else:
            self.get_logger().info("✅ Parada de Emergencia liberada.")

    # -------------------------------------------------------------
    # CALLBACK DEL PLANIFICADOR DE TRAYECTORIAS
    # -------------------------------------------------------------

    def _planner_status_callback(self, msg: Int8):
        """Notifica el estado de ejecución de la trayectoria enviada por el planificador."""
        status_code = msg.data

        if status_code == 1:
            self.get_logger().info("⏳ Trayectoria en ejecución: El planificador está enviando tramas a la STM32...")
        elif status_code == 2:
            self.get_logger().info("✅ Trayectoria completada con éxito.")
        elif status_code == -1:
            self.get_logger().error("🚨 Trayectoria cancelada, abortada o con error de alcanzabilidad.")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()