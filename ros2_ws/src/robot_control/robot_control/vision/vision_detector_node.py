#!/usr/bin/env python3
"""Nodo de visión artificial cenital para detección, velocidad y clasificación geométrica.

Procesa el flujo de video a 30 FPS en espacio de color HSV, ejecuta una
transformación métrica cuadrática (píxel a milímetro), estima la velocidad lineal
mediante compuertas espaciales fijas y clasifica la morfología de las piezas
(Cubo vs Cono) de forma invariante a la rotación combinando Extent y Rect Ratio.
"""

from typing import Optional
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge

# Ruta persistente udev para cámara cenital
SYMLINK_PATH: str = "/dev/v4l/by-id/usb-Generic_HD_camera-video-index0"


class VisionDetectorNode(Node):
    """Nodo ROS 2 para la percepción y análisis cinemático de objetos en cinta."""

    # Compuertas espaciales para cálculo de velocidad lineal determinista [mm]
    Y_GATE_1: float = -160.0  # Cota inicial de temporización [mm]
    Y_GATE_2: float = -90.0   # Cota de disparo y cálculo [mm] (Delta Y = 70.0 mm)

    # Restricciones espaciales del área de trabajo sobre la cinta [mm]
    X_FIXED_MM: float = 210.0
    Z_FIXED_MM: float = 77.0

    # Coeficientes del polinomio cuadrático calibrado: Y_robot = a*x^2 + b*x + c
    POLY_A: float = 0.0000775319
    POLY_B: float = 0.5864531721
    POLY_C: float = -185.7812192642

    def __init__(self) -> None:
        """Inicializa periféricos de captura, publicadores y estructuras morfológicas."""
        super().__init__("vision_detector_node")

        # Publicadores ROS 2
        self.publisher_estatico = self.create_publisher(
            Point, "/detected_object_pose", 10
        )
        self.publisher_dinamico = self.create_publisher(
            Point, "/detected_object_dynamic_pose", 10
        )
        self.publisher_clase = self.create_publisher(
            Int8, "/vision/object_class", 10
        )
        self.image_pub = self.create_publisher(
            Image, "/vision/image_annotated", 10
        )

        self.bridge: CvBridge = CvBridge()

        # Variables de temporización y estimación de velocidad
        self.t_gate_1: Optional[float] = None
        self.vel_calculada_mm_s: float = 0.0
        self.medicion_disparada: bool = False
        self.last_seen_time: float = 0.0

        # Apertura segura del dispositivo de video
        if os.path.exists(SYMLINK_PATH):
            camera_path = os.path.realpath(SYMLINK_PATH)
            self.get_logger().info(f"Dispositivo de video persistente: {camera_path}")
        else:
            self.get_logger().warn("Symlink no detectado. Utilizando índice por defecto 0.")
            camera_path = 0

        self.cap = cv2.VideoCapture(camera_path)
        if not self.cap.isOpened():
            self.get_logger().error("Fallo crítico al acceder a la cámara cenital.")
            return

        # Configuración del sensor a 640x480 @ 30 FPS
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Rangos cromáticos HSV calibrados para objeto de prueba (Rosa)
        self.lower_hsv: np.ndarray = np.array([131, 120, 160], dtype=np.uint8)
        self.upper_hsv: np.ndarray = np.array([179, 255, 255], dtype=np.uint8)

        # Elementos estructurantes para operaciones morfológicas
        self.kernel_close: np.ndarray = np.ones((11, 11), dtype=np.uint8)
        self.kernel_open: np.ndarray = np.ones((5, 5), dtype=np.uint8)

        # Temporizador periódico de procesamiento a ~30 Hz (33 ms)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info("Nodo de visión continua en tiempo real activo.")

    def transform_px_to_mm_quadratic(self, cx_px: float) -> float:
        """Traduce la coordenada centroidal en píxeles a posición longitudinal real.

        Args:
            cx_px (float): Coordenada centroidal X en píxeles del contorno [px].

        Returns:
            float: Coordenada Y proyectada sobre el eje del robot [mm].
        """
        return float(
            (self.POLY_A * (cx_px ** 2)) + (self.POLY_B * cx_px) + self.POLY_C
        )

    def process_frame(self) -> None:
        """Bucle principal de procesamiento digital de imágenes y publicación."""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        now = time.perf_counter()

        # Reset automático del estado si la pieza abandonó la ROI por más de 0.6 s
        if self.t_gate_1 is not None and (now - self.last_seen_time > 0.6):
            self.t_gate_1 = None
            self.medicion_disparada = False
            self.vel_calculada_mm_s = 0.0

        # Segmentación y filtrado morfológico en espacio de color HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area_objeto = cv2.contourArea(largest)

            if area_objeto > 80:
                self.last_seen_time = now

                # 1. Bounding Box recto (ejes de imagen)
                rect_upright = cv2.boundingRect(largest)
                area_upright = rect_upright[2] * rect_upright[3]

                # 2. Rectángulo orientado de área mínima
                rect_rot = cv2.minAreaRect(largest)
                w_rot, h_rot = rect_rot[1]

                forma_txt = "Objeto"
                if w_rot != 0 and h_rot != 0 and area_upright != 0:
                    area_rot = w_rot * h_rot
                    extent_rotado = float(area_objeto) / area_rot
                    ratio_cajas = area_rot / float(area_upright)

                    clase_msg = Int8()
                    # Si ratio_cajas < 0.85 -> rotado sobre la cinta (Cubo)
                    # Si ratio_cajas >= 0.85 -> alineado; se discrimina por densidad (Extent > 0.85 = Cubo)
                    if ratio_cajas < 0.85 or extent_rotado > 0.85:
                        clase_msg.data = 1
                        forma_txt = "Cubo"
                    else:
                        clase_msg.data = 2
                        forma_txt = "Cono"

                    self.publisher_clase.publish(clase_msg)

                # Cálculo del centroide mediante momentos de imagen
                moments = cv2.moments(largest)
                if moments["m00"] != 0:
                    cx = float(moments["m10"] / moments["m00"])
                    cy = float(moments["m01"] / moments["m00"])
                    y_robot = self.transform_px_to_mm_quadratic(cx)

                    # Publicación de coordenadas estáticas continuas
                    msg_est = Point()
                    msg_est.x = self.X_FIXED_MM
                    msg_est.y = y_robot
                    msg_est.z = self.Z_FIXED_MM
                    self.publisher_estatico.publish(msg_est)

                    # Estimación determinista de velocidad entre compuertas espaciales
                    if not self.medicion_disparada:
                        if self.t_gate_1 is None:
                            if y_robot >= self.Y_GATE_1:
                                self.t_gate_1 = now
                        else:
                            if y_robot >= self.Y_GATE_2:
                                dt = now - self.t_gate_1
                                if dt > 0.1:
                                    dy = y_robot - self.Y_GATE_1
                                    self.vel_calculada_mm_s = dy / dt
                                    self.medicion_disparada = True
                                    self.get_logger().info(
                                        f"Velocidad lineal estimada: {self.vel_calculada_mm_s:.1f} mm/s"
                                    )

                    # Publicación de paquete dinámico sincronizado
                    msg_dyn = Point()
                    msg_dyn.x = float(self.vel_calculada_mm_s)
                    msg_dyn.y = float(y_robot)
                    msg_dyn.z = 1.0 if self.medicion_disparada else 0.0
                    self.publisher_dinamico.publish(msg_dyn)

                    # Anotaciones gráficas en el fotograma para depuración visual
                    if w_rot != 0 and h_rot != 0 and area_upright != 0:
                        box = np.intp(cv2.boxPoints(rect_rot))
                        cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    txt = f"{forma_txt} | Y:{y_robot:.1f} mm"
                    cv2.putText(
                        frame,
                        txt,
                        (int(cx) + 10, int(cy) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception:
            pass

    def destroy_node(self) -> None:
        """Libera de forma segura la captura de video y ventanas activas."""
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Punto de entrada de ejecución del nodo de visión."""
    rclpy.init(args=args)
    node = VisionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()