#!/usr/bin/env python3
"""Punto de entrada de la interfaz gráfica V2 para intercepción dinámica en 1 paso."""

import sys
from typing import List, Optional
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QSlider,
    QLabel,
    QGroupBox,
    QGridLayout,
    QTabWidget,
    QDoubleSpinBox,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPixmap, QImage

from robot_control.gui.ros_thread import ROS2Thread
from robot_control.gui_pickAndPlace_Final.dynamic_continuous_tab import (
    DynamicContinuousTab,
)
from robot_control.gui_pickAndPlace_Final.pick_and_place_sm_v2 import (
    PickAndPlaceSMV2,
)


class MainWindowV2(QMainWindow):
    """Ventana de control y monitoreo simplificada para flujo continuo en 1 paso."""

    def __init__(self, ros_thread: ROS2Thread) -> None:
        """Inicializa componentes visuales y enlaces de comunicación con ROS 2."""
        super().__init__()
        self.ros_thread = ros_thread

        self.estop_active: bool = False
        self.magnet_active: bool = False

        self.q_real: List[float] = [0.0, 90.0, 0.0]
        self.pos_xyz: List[float] = [170.0, 0.0, 170.0]
        self.q_target: List[float] = [0.0, 90.0, 0.0]

        # Conexión de señales provenientes del hilo ROS2Thread
        self.ros_thread.feedback_received.connect(self.on_feedback_received)
        self.ros_thread.planner_status_received.connect(
            self.on_planner_status_received
        )
        self.ros_thread.vision_dynamic_target_received.connect(
            self.on_vision_dynamic_target_received
        )
        self.ros_thread.vision_image_received.connect(
            self.on_vision_image_received
        )

        if hasattr(self.ros_thread, "vision_class_received"):
            self.ros_thread.vision_class_received.connect(
                self.on_vision_class_received
            )

        self.init_ui()
        self.sm = PickAndPlaceSMV2(self.ros_thread, self.tab_dynamic_v2)

    def on_vision_class_received(self, clase: int) -> None:
        self.sm.set_object_class(clase)

    def on_vision_dynamic_target_received(
        self, target_xyz: List[float]
    ) -> None:
        vel_mm_s, y_actual, flag_medido = target_xyz
        self.sm.process_dynamic_vision(vel_mm_s, y_actual, flag_medido)

    def execute_interception(self) -> None:
        """Dispara el modo dinámico continuo desde la posición configurada en la GUI."""
        if self.estop_active:
            return
        guard_pos = [
            self.tab_dynamic_v2.spin_gx.value(),
            self.tab_dynamic_v2.spin_gy.value(),
            self.tab_dynamic_v2.spin_gz.value(),
        ]
        self.sm.start_dynamic_mode(initial_pos=guard_pos)

    def on_planner_status_received(self, code: int) -> None:
        self.sm.process_planner_status(code, self)

    def init_ui(self) -> None:
        """Construye las pestañas de control manual y operación continua V2."""
        self.setWindowTitle(
            "Control y Monitoreo V2 - Pick-and-Place Continuo (STM32 Embedded)"
        )
        self.resize(780, 920)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tab_control = QWidget()
        self.setup_tab_control()
        self.tabs.addTab(self.tab_control, "Control Manual (P2P)")

        self.tab_dynamic_v2 = DynamicContinuousTab()
        self.tabs.addTab(self.tab_dynamic_v2, "Dinámico Continuo (1-Paso)")
        self.tab_dynamic_v2.btn_dyn_start.clicked.connect(self.execute_interception)
        self.tab_dynamic_v2.btn_dyn_estop.clicked.connect(self.toggle_estop)

    def setup_tab_control(self) -> None:
        layout = QVBoxLayout(self.tab_control)

        self.btn_estop = QPushButton("PARADA DE EMERGENCIA (INACTIVA)")
        self.btn_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_estop.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;"
        )
        self.btn_estop.clicked.connect(self.toggle_estop)
        layout.addWidget(self.btn_estop)

        group_indep = QGroupBox("Acciones Independientes")
        layout_indep = QHBoxLayout()
        self.btn_homing = QPushButton("EJECUTAR HOMING")
        self.btn_homing.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_homing.setStyleSheet(
            "background-color: #2980b9; color: white; padding: 10px; border-radius: 5px;"
        )
        self.btn_homing.clicked.connect(lambda: self.ros_thread.send_homing())

        self.btn_magnet = QPushButton("ELECTROIMÁN: APAGADO")
        self.btn_magnet.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_magnet.setStyleSheet(
            "background-color: #7f8c8d; color: white; padding: 10px; border-radius: 5px;"
        )
        self.btn_magnet.clicked.connect(self.toggle_magnet)

        layout_indep.addWidget(self.btn_homing)
        layout_indep.addWidget(self.btn_magnet)
        group_indep.setLayout(layout_indep)
        layout.addWidget(group_indep)

        group_motors = QGroupBox("Ajuste Articular Directo (Modo Manual P2P)")
        layout_motors = QVBoxLayout()
        grid_sliders = QGridLayout()

        grid_sliders.addWidget(QLabel("Q1 [-90° a 90°]:"), 0, 0)
        self.slider_q1 = QSlider(Qt.Horizontal)
        self.slider_q1.setRange(-90, 90)
        self.slider_q1.setValue(0)
        self.spin_q1 = QDoubleSpinBox()
        self.spin_q1.setRange(-90.0, 90.0)
        self.spin_q1.setValue(0.0)
        self.spin_q1.setSingleStep(0.5)
        self.slider_q1.valueChanged.connect(
            lambda v: self.spin_q1.setValue(float(v))
        )
        self.spin_q1.valueChanged.connect(
            lambda v: self.slider_q1.setValue(int(v))
        )
        grid_sliders.addWidget(self.slider_q1, 0, 1)
        grid_sliders.addWidget(self.spin_q1, 0, 2)

        grid_sliders.addWidget(QLabel("Q2 [40° a 157°]:"), 1, 0)
        self.slider_q2 = QSlider(Qt.Horizontal)
        self.slider_q2.setRange(40, 157)
        self.slider_q2.setValue(90)
        self.spin_q2 = QDoubleSpinBox()
        self.spin_q2.setRange(40.0, 157.0)
        self.spin_q2.setValue(90.0)
        self.spin_q2.setSingleStep(0.5)
        self.slider_q2.valueChanged.connect(
            lambda v: self.spin_q2.setValue(float(v))
        )
        self.spin_q2.valueChanged.connect(
            lambda v: self.slider_q2.setValue(int(v))
        )
        grid_sliders.addWidget(self.slider_q2, 1, 1)
        grid_sliders.addWidget(self.spin_q2, 1, 2)

        grid_sliders.addWidget(QLabel("Q3 [-75° a 25°]:"), 2, 0)
        self.slider_q3 = QSlider(Qt.Horizontal)
        self.slider_q3.setRange(-75, 25)
        self.slider_q3.setValue(0)
        self.spin_q3 = QDoubleSpinBox()
        self.spin_q3.setRange(-75.0, 25.0)
        self.spin_q3.setValue(0.0)
        self.spin_q3.setSingleStep(0.5)
        self.slider_q3.valueChanged.connect(
            lambda v: self.spin_q3.setValue(float(v))
        )
        self.spin_q3.valueChanged.connect(
            lambda v: self.slider_q3.setValue(int(v))
        )
        grid_sliders.addWidget(self.slider_q3, 2, 1)
        grid_sliders.addWidget(self.spin_q3, 2, 2)

        layout_motors.addLayout(grid_sliders)

        self.btn_move_motors = QPushButton(
            "ENVIAR ÁNGULOS ARTICULARES (P2P)"
        )
        self.btn_move_motors.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_move_motors.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;"
        )
        self.btn_move_motors.clicked.connect(self.send_joint_cmd)

        layout_motors.addWidget(self.btn_move_motors)
        group_motors.setLayout(layout_motors)
        layout.addWidget(group_motors)

        group_cartesian_ctrl = QGroupBox(
            "Ajuste Cartesiano Directo (P2P / Coordenadas X,Y,Z)"
        )
        layout_cart_ctrl = QVBoxLayout()
        grid_xyz = QGridLayout()

        grid_xyz.addWidget(QLabel("X [mm]:"), 0, 0)
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(0.0, 300.0)
        self.spin_x.setValue(170.0)
        self.spin_x.setSingleStep(5.0)
        grid_xyz.addWidget(self.spin_x, 0, 1)

        grid_xyz.addWidget(QLabel("Y [mm]:"), 0, 2)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-200.0, 200.0)
        self.spin_y.setValue(0.0)
        self.spin_y.setSingleStep(5.0)
        grid_xyz.addWidget(self.spin_y, 0, 3)

        grid_xyz.addWidget(QLabel("Z [mm]:"), 0, 4)
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(20.0, 280.0)
        self.spin_z.setValue(170.0)
        self.spin_z.setSingleStep(5.0)
        grid_xyz.addWidget(self.spin_z, 0, 5)

        layout_cart_ctrl.addLayout(grid_xyz)

        self.btn_move_cartesian = QPushButton(
            "ENVIAR CONSIGNA CARTESIANA (IK P2P)"
        )
        self.btn_move_cartesian.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_move_cartesian.setStyleSheet(
            "background-color: #8e44ad; color: white; padding: 10px; border-radius: 5px;"
        )
        self.btn_move_cartesian.clicked.connect(self.send_cartesian_cmd)

        layout_cart_ctrl.addWidget(self.btn_move_cartesian)
        group_cartesian_ctrl.setLayout(layout_cart_ctrl)
        layout.addWidget(group_cartesian_ctrl)

        font_bold = QFont("Arial", 10, QFont.Bold)

        group_fb = QGroupBox("Ángulos Reales (Sensores AS5600)")
        grid_fb = QGridLayout()
        self.lbl_q1_real = QLabel("Q1: 0.00°")
        self.lbl_q2_real = QLabel("Q2: 90.00°")
        self.lbl_q3_real = QLabel("Q3: 0.00°")
        self.lbl_q1_real.setFont(font_bold)
        self.lbl_q2_real.setFont(font_bold)
        self.lbl_q3_real.setFont(font_bold)
        grid_fb.addWidget(self.lbl_q1_real, 0, 0, Qt.AlignCenter)
        grid_fb.addWidget(self.lbl_q2_real, 0, 1, Qt.AlignCenter)
        grid_fb.addWidget(self.lbl_q3_real, 0, 2, Qt.AlignCenter)
        group_fb.setLayout(grid_fb)
        layout.addWidget(group_fb)

        group_cart = QGroupBox("Posición Cartesiana (FK)")
        grid_cart = QGridLayout()
        self.lbl_x_real = QLabel("X: 0.00 mm")
        self.lbl_y_real = QLabel("Y: 0.00 mm")
        self.lbl_z_real = QLabel("Z: 0.00 mm")
        self.lbl_x_real.setFont(font_bold)
        self.lbl_y_real.setFont(font_bold)
        self.lbl_z_real.setFont(font_bold)
        self.lbl_x_real.setStyleSheet("color: #27ae60;")
        self.lbl_y_real.setStyleSheet("color: #2980b9;")
        self.lbl_z_real.setStyleSheet("color: #8e44ad;")
        grid_cart.addWidget(self.lbl_x_real, 0, 0, Qt.AlignCenter)
        grid_cart.addWidget(self.lbl_y_real, 0, 1, Qt.AlignCenter)
        grid_cart.addWidget(self.lbl_z_real, 0, 2, Qt.AlignCenter)
        group_cart.setLayout(grid_cart)
        layout.addWidget(group_cart)

    def on_vision_image_received(self, q_img: QImage) -> None:
        if q_img.isNull():
            return
        pixmap = QPixmap.fromImage(q_img)
        target_size = self.tab_dynamic_v2.lbl_camera_dyn.size()
        if target_size.width() > 10 and target_size.height() > 10:
            self.tab_dynamic_v2.lbl_camera_dyn.setPixmap(
                pixmap.scaled(
                    target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
            )
        else:
            self.tab_dynamic_v2.lbl_camera_dyn.setPixmap(pixmap)

    def send_joint_cmd(self) -> None:
        if self.estop_active:
            return
        q1_deg = float(self.spin_q1.value())
        q2_deg = float(self.spin_q2.value())
        q3_deg = float(self.spin_q3.value())
        self.q_target = [q1_deg, q2_deg, q3_deg]
        self.ros_thread.send_cmd(q1_deg, q2_deg, q3_deg)

    def send_cartesian_cmd(self) -> None:
        if self.estop_active:
            return
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        success, q_target = self.ros_thread.send_cartesian_cmd(x, y, z)
        if success:
            self.q_target = q_target
            self.spin_q1.setValue(q_target[0])
            self.spin_q2.setValue(q_target[1])
            self.spin_q3.setValue(q_target[2])

    def toggle_magnet(self) -> None:
        self.magnet_active = not self.magnet_active
        self.ros_thread.send_magnet(self.magnet_active)
        self.btn_magnet.setText(
            "ELECTROIMÁN: ENCENDIDO" if self.magnet_active else "ELECTROIMÁN: APAGADO"
        )

    def toggle_estop(self) -> None:
        self.estop_active = not self.estop_active
        self.ros_thread.send_estop(self.estop_active)
        self.sm.set_estop(self.estop_active)

        btn_text = (
            "PARADA DE EMERGENCIA ACTIVADA"
            if self.estop_active
            else "PARADA DE EMERGENCIA (INACTIVA)"
        )
        self.btn_estop.setText(btn_text)
        self.tab_dynamic_v2.btn_dyn_estop.setText(btn_text)

    def on_feedback_received(
        self, q_real: List[float], pos_xyz: List[float]
    ) -> None:
        self.q_real = q_real
        self.pos_xyz = pos_xyz
        self.sm.set_robot_pos(pos_xyz)

        self.lbl_q1_real.setText(f"Q1: {q_real[0]:.2f}°")
        self.lbl_q2_real.setText(f"Q2: {q_real[1]:.2f}°")
        self.lbl_q3_real.setText(f"Q3: {q_real[2]:.2f}°")
        self.lbl_x_real.setText(f"X: {pos_xyz[0]:.2f} mm")
        self.lbl_y_real.setText(f"Y: {pos_xyz[1]:.2f} mm")
        self.lbl_z_real.setText(f"Z: {pos_xyz[2]:.2f} mm")

        v_q_txt = f"Q1={q_real[0]:.1f}  Q2={q_real[1]:.1f}  Q3={q_real[2]:.1f}"
        v_xyz_txt = f"X={pos_xyz[0]:.1f}  Y={pos_xyz[1]:.1f}  Z={pos_xyz[2]:.1f}"

        self.tab_dynamic_v2.lbl_dyn_q.setText(v_q_txt)
        self.tab_dynamic_v2.lbl_dyn_xyz.setText(v_xyz_txt)


def main(args: Optional[list] = None) -> None:
    """Punto de arranque de la GUI V2."""
    app = QApplication(sys.argv)
    ros_thread = ROS2Thread()
    ros_thread.start()

    window = MainWindowV2(ros_thread)
    window.show()

    exit_code = app.exec_()
    ros_thread.quit()
    ros_thread.wait()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()