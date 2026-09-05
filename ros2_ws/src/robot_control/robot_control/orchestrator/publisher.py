#!/usr/bin/env python3
"""Nodo puente de interfaz y orquestación de hardware entre ROS 2 y micro-ROS.

Centraliza el enrutamiento de consignas de movimiento (P2P), calibración
de origen (homing), control del efector final magnético y difusión
prioritaria de paradas de emergencia (E-Stop).
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Point


class MinimalPublisher(Node):
    """Nodo orquestador y puente de hardware para la celda de manufactura.

    Administra la coherencia del estado del sistema, asegurando que las
    consignas provenientes de la interfaz gráfica solo alcancen la capa
    embebida si las condiciones de seguridad lo permiten.
    """

    def __init__(self) -> None:
        """Inicializa publicadores, suscriptores y estados de seguridad."""
        super().__init__("minimal_publisher")

        self.is_estop_active: bool = False

        # --- PUBLICADORES HACIA MICROROS (STM32 HARDWARE INTERFACE) ---
        self.micro_p2p_pub = self.create_publisher(Point, "/microROS/p2p_cmd", 10)
        self.micro_homing_pub = self.create_publisher(Bool, "/microROS/homing", 10)
        self.micro_magnet_pub = self.create_publisher(
            Bool, "/microROS/electroiman", 10
        )
        self.micro_estop_pub = self.create_publisher(
            Bool, "/microROS/emergency_stop", 10
        )

        # --- PUBLICADORES HACIA NODOS SECUNDARIOS ROS 2 ---
        self.planner_estop_pub = self.create_publisher(
            Bool, "/planner/emergency_stop", 10
        )

        # --- SUSCRIPTORES DESDE LA INTERFAZ DE USUARIO (GUI) ---
        self.ui_homing_sub = self.create_subscription(
            Bool, "/robot_ui/homing", self._ui_homing_callback, 10
        )
        self.ui_magnet_sub = self.create_subscription(
            Bool, "/robot_ui/e_magnet_on", self._ui_magnet_callback, 10
        )
        self.ui_estop_sub = self.create_subscription(
            Bool, "/robot_ui/emergency_stop", self._ui_estop_callback, 10
        )
        self.ui_p2p_sub = self.create_subscription(
            Point, "/robot_ui/p2p_cmd", self._ui_p2p_callback, 10
        )

        # --- SUSCRIPTORES DESDE EL PLANIFICADOR DE TRAYECTORIAS EMBEBIDO ---
        self.planner_status_sub = self.create_subscription(
            Int8, "/planner/traj_status", self._planner_status_callback, 10
        )

        self.get_logger().info(
            "Orquestador Central (Hardware Bridge) inicializado correctamente."
        )

    # -------------------------------------------------------------
    # CALLBACKS DE INTERFAZ DE USUARIO
    # -------------------------------------------------------------

    def _ui_p2p_callback(self, msg: Point) -> None:
        """Procesa y reenvía comandos articulares directos a la STM32.

        Args:
            msg (Point): Coordenadas articulares [Q1, Q2, Q3] en grados sexagesimales [°].
        """
        if self.is_estop_active:
            self.get_logger().warn("E-Stop activo: Consigna P2P rechazada.")
            return

        self.get_logger().info(
            f"Consigna articular P2P enviada -> Q1={msg.x:.1f}°, Q2={msg.y:.1f}°, Q3={msg.z:.1f}°"
        )
        self.micro_p2p_pub.publish(msg)

    def _ui_homing_callback(self, msg: Bool) -> None:
        """Dispara la secuencia global de referenciamiento mecánico en el firmware.

        Args:
            msg (Bool): Bandera booleana de activación de homing.
        """
        if msg.data and not self.is_estop_active:
            self.get_logger().info("Orden de Homing transmitida a micro-ROS...")
            self.micro_homing_pub.publish(msg)

    def _ui_magnet_callback(self, msg: Bool) -> None:
        """Enruta el estado de excitación del electroimán hacia el driver embebido.

        Args:
            msg (Bool): Estado booleano del solenoide (True = On, False = Off).
        """
        self.micro_magnet_pub.publish(msg)
        estado_str = "ENCENDIDO" if msg.data else "APAGADO"
        self.get_logger().info(f"Orden de efector magnético enviada: {estado_str}")

    def _ui_estop_callback(self, msg: Bool) -> None:
        """Propaga el estado prioritario de parada de emergencia en el bus distribuido.

        Args:
            msg (Bool): Bandera booleana de interrupción de emergencia.
        """
        self.is_estop_active = msg.data
        self.micro_estop_pub.publish(msg)
        self.planner_estop_pub.publish(msg)

        if self.is_estop_active:
            self.get_logger().error("PARADA DE EMERGENCIA ACTIVADA EN EL ORQUESTADOR.")
        else:
            self.get_logger().info("Parada de Emergencia liberada.")

    # -------------------------------------------------------------
    # CALLBACK DE RETROALIMENTACIÓN DE PLANIFICACIÓN
    # -------------------------------------------------------------

    def _planner_status_callback(self, msg: Int8) -> None:
        """Supervisa los códigos de retorno de la máquina de estados del firmware.

        Args:
            msg (Int8): Código de estado numérico emitido por la STM32:
                        1 = Trayectoria en progreso.
                        2 = Asentamiento y trayectoria completados.
                       -1 = Rechazo cinemático o error dinámico.
        """
        status_code = msg.data
        if status_code == 1:
            self.get_logger().info("Trayectoria quíntica en ejecución en STM32...")
        elif status_code == 2:
            self.get_logger().info("Trayectoria completada con éxito.")
        elif status_code == -1:
            self.get_logger().error("Fallo cinemático o aborto de trayectoria en STM32.")


def main(args: Optional[list] = None) -> None:
    """Punto de entrada de ejecución del nodo orquestador."""
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()