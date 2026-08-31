# Archivo: robot_control/vision/vision_detector_node.py
#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import time

SYMLINK_PATH = '/dev/v4l/by-id/usb-Generic_HD_camera-video-index0'


class VisionDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_detector_node')

        self.publisher_estico = self.create_publisher(Point, '/detected_object_pose', 10)
        self.publisher_dinamico = self.create_publisher(Point, '/detected_object_dynamic_pose', 10)
        self.publisher_clase = self.create_publisher(Int8, '/vision/object_class', 10)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/vision/image_annotated', 10)
        
        # Líneas de corte fijas para medición precisa de 80 mm
        self.Y_GATE_1 = -160.0         # Cruce inicial
        self.Y_GATE_2 = -90.0          # Cruce final y disparo
        self.t_gate_1 = None
        self.vel_calculada_mm_s = 0.0
        self.medicion_disparada = False
        self.last_seen_time = 0.0

        if os.path.exists(SYMLINK_PATH):
            camera_path = os.path.realpath(SYMLINK_PATH)
            self.get_logger().info(f"🔗 Symlink encontrado: {camera_path}")
        else:
            self.get_logger().warn("⚠️ No se encontró el symlink. Usando índice 0.")
            camera_path = 0

        self.cap = cv2.VideoCapture(camera_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ No se pudo acceder a la cámara.")
            return

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # RANGOS HSV
        self.lower_hsv = np.array([136, 64, 219])
        self.upper_hsv = np.array([179, 255, 255])

        self.kernel_close = np.ones((11, 11), np.uint8)
        self.kernel_open = np.ones((5, 5), np.uint8)

        self.x_fixed_mm = 210.0
        self.z_fixed_mm = 77.0

        # Coeficientes de calibración cuadrática
        self.poly_a = 0.0000775319
        self.poly_b = 0.5864531721
        self.poly_c = -185.7812192642

        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info("✅ Nodo de visión continua listo.")

    def transform_px_to_mm_quadratic(self, cx_px):
        return float((self.poly_a * (cx_px ** 2)) + (self.poly_b * cx_px) + self.poly_c)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        now = time.perf_counter()

        # Reset automático tras salir el objeto
        if self.t_gate_1 is not None and (now - self.last_seen_time > 0.6):
            self.t_gate_1 = None
            self.medicion_disparada = False
            self.vel_calculada_mm_s = 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area_objeto = cv2.contourArea(largest)
            
            if area_objeto > 80:
                self.last_seen_time = now

                # 1. Caja recta (Upright Bounding Box)
                rect_upright = cv2.boundingRect(largest)
                area_upright = rect_upright[2] * rect_upright[3]
                
                # 2. Caja rotada (Min Area Rect)
                rect_rot = cv2.minAreaRect(largest)
                w_rot, h_rot = rect_rot[1]
                
                if w_rot != 0 and h_rot != 0 and area_upright != 0:
                    area_rot = w_rot * h_rot
                    
                    # 3. Métricas Geométricas
                    extent_rotado = float(area_objeto) / area_rot
                    ratio_cajas = area_rot / area_upright
                    
                    clase_msg = Int8()
                    
                    # 4. Lógica de Clasificación Robusta
                    if ratio_cajas < 0.85 or extent_rotado > 0.85:
                        clase_msg.data = 1
                        forma_txt = f"Cubo (E:{extent_rotado:.2f} R:{ratio_cajas:.2f})"
                    else:
                        clase_msg.data = 2
                        forma_txt = f"Cono (E:{extent_rotado:.2f} R:{ratio_cajas:.2f})"
                    
                    self.publisher_clase.publish(clase_msg)

                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = float(M["m10"] / M["m00"])
                    cy = float(M["m01"] / M["m00"])
                    y_robot = self.transform_px_to_mm_quadratic(cx)

                    # Publicación continua de coordenadas
                    msg_est = Point()
                    msg_est.x = self.x_fixed_mm
                    msg_est.y = y_robot
                    msg_est.z = self.z_fixed_mm
                    self.publisher_estico.publish(msg_est)

                    # Medición determinista por cruce de 2 líneas fijas (-160 mm -> -80 mm)
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
                                    self.get_logger().info(f"🎯 Velocidad exacta calculada: {self.vel_calculada_mm_s:.1f} mm/s")

                    # Publicación de paquete sincronizado
                    msg_dyn = Point()
                    msg_dyn.x = float(self.vel_calculada_mm_s)
                    msg_dyn.y = float(y_robot)
                    msg_dyn.z = 1.0 if self.medicion_disparada else 0.0
                    self.publisher_dinamico.publish(msg_dyn)

                    # Dibujo en cámara
                    if w_rot != 0 and h_rot != 0 and area_upright != 0:
                        box = np.intp(cv2.boxPoints(rect_rot))
                        cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    if 'forma_txt' in locals():
                        txt = f"{forma_txt} | Y:{y_robot:.1f} | V:{self.vel_calculada_mm_s:.1f}"
                        cv2.putText(frame, txt, (int(cx) + 10, int(cy) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception:
            pass

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
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


if __name__ == '__main__':
    main()