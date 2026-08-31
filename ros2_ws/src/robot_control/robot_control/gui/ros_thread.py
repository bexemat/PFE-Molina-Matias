#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage

from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from extra_interfaces.msg import Trama

from robot_control.gui.kinematics import forward_kinematics, inverse_kinematics


class RobotUINode(Node):
    """Nodo ROS 2 que enlaza la UI directamente con la STM32 (micro-ROS) y el nodo de visión."""
    
    def __init__(self, feedback_signal: pyqtSignal, status_signal: pyqtSignal, 
                 vision_target_signal: pyqtSignal, vision_dynamic_target_signal: pyqtSignal, 
                 vision_image_signal: pyqtSignal, vision_class_signal: pyqtSignal):
        super().__init__('robot_ui_node')
        
        self.feedback_signal = feedback_signal
        self.status_signal = status_signal
        self.vision_target_signal = vision_target_signal
        self.vision_dynamic_target_signal = vision_dynamic_target_signal
        self.vision_image_signal = vision_image_signal
        self.vision_class_signal = vision_class_signal

        # --- PUBLICADORES DIRECTOS HACIA LA STM32 ---
        self.ctraj_pub = self.create_publisher(Trama, '/microROS/cmd', 10)
        self.p2p_cmd_pub = self.create_publisher(Point, '/microROS/p2p_cmd', 10)
        self.homing_pub = self.create_publisher(Bool, '/microROS/homing', 10)
        self.magnet_pub = self.create_publisher(Bool, '/microROS/electroiman', 10)
        self.estop_pub = self.create_publisher(Bool, '/microROS/emergency_stop', 10)

        # --- PUBLICADOR HACIA EL NODO DE VISIÓN ---
        self.trigger_meas_pub = self.create_publisher(Bool, '/vision/trigger_measurement', 10)

        # --- SUSCRIPTORES DESDE LA STM32 ---
        self.feedback_sub = self.create_subscription(
            Point, '/microROS/angles', self._feedback_callback, 10
        )
        self.status_sub = self.create_subscription(
            Int8, '/planner/traj_status', self._status_callback, 10
        )

        # --- SUSCRIPTORES DESDE EL NODO DE VISIÓN ---
        self.vision_target_sub = self.create_subscription(
            Point, '/detected_object_pose', self._vision_target_callback, 10
        )
        self.vision_dynamic_sub = self.create_subscription(
            Point, '/detected_object_dynamic_pose', self._vision_dynamic_callback, 10
        )
        self.vision_image_sub = self.create_subscription(
            Image, '/vision/image_annotated', self._vision_image_callback, 10
        )
        self.vision_class_sub = self.create_subscription(
            Int8, '/vision/object_class', self._vision_class_callback, 10
        )

        self.get_logger().info("✅ Nodo UI conectado directamente a micro-ROS (STM32) y Visión.")

    def _feedback_callback(self, msg: Point):
        q_real = [round(msg.x, 2), round(msg.y, 2), round(msg.z, 2)]
        x_mm, y_mm, z_mm, _ = forward_kinematics(q_real)
        pos_xyz = [round(x_mm, 2), round(y_mm, 2), round(z_mm, 2)]
        self.feedback_signal.emit(q_real, pos_xyz)

    def _status_callback(self, msg: Int8):
        self.status_signal.emit(int(msg.data))

    def _vision_target_callback(self, msg: Point):
        self.vision_target_signal.emit([msg.x, msg.y, msg.z])

    def _vision_dynamic_callback(self, msg: Point):
        self.vision_dynamic_target_signal.emit([msg.x, msg.y, msg.z])

    def _vision_class_callback(self, msg: Int8):
        self.vision_class_signal.emit(int(msg.data))

    def _vision_image_callback(self, msg: Image):
        try:
            image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            h, w, ch = image_np.shape
            bytes_per_line = ch * w
            # .copy() es obligatorio para evitar que el garbage collector libere el buffer
            q_img = QImage(image_np.data, w, h, bytes_per_line, QImage.Format_BGR888).copy()
            self.vision_image_signal.emit(q_img)
        except Exception as e:
            pass


class ROS2Thread(QThread):
    """Hilo secundario para la ejecución del spin de ROS 2 sin congelar PyQt5."""
    
    feedback_received = pyqtSignal(list, list)
    planner_status_received = pyqtSignal(int)
    vision_target_received = pyqtSignal(list)
    vision_dynamic_target_received = pyqtSignal(list)
    vision_image_received = pyqtSignal(QImage)
    vision_class_received = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.node = None

    def run(self):
        rclpy.init()
        self.node = RobotUINode(
            self.feedback_received, 
            self.planner_status_received,
            self.vision_target_received,
            self.vision_dynamic_target_received,
            self.vision_image_received,
            self.vision_class_received
        )
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def send_cmd(self, q1_deg: float, q2_deg: float, q3_deg: float):
        if not self.node: return
        msg = Point()
        msg.x = float(q1_deg)
        msg.y = float(q2_deg)
        msg.z = float(q3_deg)
        self.node.p2p_cmd_pub.publish(msg)

    def send_cartesian_cmd(self, x_mm: float, y_mm: float, z_mm: float):
        q_target, reachable = inverse_kinematics(x_mm, y_mm, z_mm)
        if reachable:
            self.send_cmd(q_target[0], q_target[1], q_target[2])
            return True, q_target
        return False, [0.0, 0.0, 0.0]

    def send_ctraj_cmd(self, x_mm: float, y_mm: float, z_mm: float, duration: float = 1.2):
        if not self.node: return
        msg = Trama()
        msg.q = [float(x_mm), float(y_mm), float(z_mm)]
        msg.qd = [0.0, 0.0, 0.0]
        msg.t_total = float(duration)
        msg.n_iter = 0
        msg.traj_state = 1
        self.node.ctraj_pub.publish(msg)

    def send_sync_ctraj_cmd(self, t_llegada: float, y_mm: float, z_mm: float):
        duracion = max(0.8, float(t_llegada))
        self.send_ctraj_cmd(220.0, y_mm, z_mm, duration=duracion)

    def send_homing(self):
        if self.node:
            msg = Bool()
            msg.data = True
            self.node.homing_pub.publish(msg)

    def send_magnet(self, state: bool):
        if self.node:
            msg = Bool()
            msg.data = state
            self.node.magnet_pub.publish(msg)

    def send_estop(self, state: bool):
        if self.node:
            msg = Bool()
            msg.data = state
            self.node.estop_pub.publish(msg)

    def send_trigger_measurement(self):
        if self.node:
            msg = Bool()
            msg.data = True
            self.node.trigger_meas_pub.publish(msg)