#!/usr/bin/env python3
import sys
import time
import collections
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QSlider, QLabel, QGroupBox, QGridLayout, QTabWidget, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap, QImage

from robot_control.gui.widgets.mpl_canvas import MplCanvas
from robot_control.gui.widgets.mpl_canvas_yz import MplCanvas2D_YZ
from robot_control.gui.tabs.dynamic_tab import DynamicTab
from robot_control.gui.ros_thread import ROS2Thread

# --- IMPORTAMOS LA NUEVA MÁQUINA DE ESTADOS MODULARIZADA ---
from robot_control.gui.pick_and_place_sm import PickAndPlaceSM


class MainWindow(QMainWindow):
    def __init__(self, ros_thread: ROS2Thread):
        super().__init__()
        self.ros_thread = ros_thread

        # Variables de estado de la UI
        self.estop_active = False
        self.magnet_active = False

        self.q_real = [0.0, 90.0, 0.0]
        self.pos_xyz = [170.0, 0.0, 170.0]
        self.q_target = [0.0, 90.0, 0.0]

        # Historiales para los gráficos
        self.max_points = 100
        self.time_history = collections.deque(maxlen=self.max_points)
        self.q1_real_hist = collections.deque(maxlen=self.max_points)
        self.q2_real_hist = collections.deque(maxlen=self.max_points)
        self.q3_real_hist = collections.deque(maxlen=self.max_points)
        
        self.q1_target_hist = collections.deque(maxlen=self.max_points)
        self.q2_target_hist = collections.deque(maxlen=self.max_points)
        self.q3_target_hist = collections.deque(maxlen=self.max_points)

        self.start_time = time.time()

        # Variables de visión
        self.vision_target = [0.0, 0.0, 0.0]
        self.vision_start_y = None
        self.vision_start_z = None
        self.vision_target_y = None
        self.vision_target_z = None
        self.yz_trace_y = []
        self.yz_trace_z = []
        self.is_tracking_yz = False  

        self.waiting_for_speed = False
        self.last_detection_time = 0.0

        # Conexiones de retroalimentación de ROS
        self.ros_thread.feedback_received.connect(self.on_feedback_received)
        self.ros_thread.planner_status_received.connect(self.on_planner_status_received)
        self.ros_thread.vision_target_received.connect(self.on_vision_target_received)
        self.ros_thread.vision_dynamic_target_received.connect(self.on_vision_dynamic_target_received)
        self.ros_thread.vision_image_received.connect(self.on_vision_image_received)
        
        if hasattr(self.ros_thread, 'vision_class_received'):
            self.ros_thread.vision_class_received.connect(self.on_vision_class_received)
        
        self.init_ui()

        # --- INSTANCIAMOS LA MÁQUINA DE ESTADOS MODULARIZADA ---
        self.sm = PickAndPlaceSM(self.ros_thread, self.tab_dynamic)

    # =================================================================
    # COMUNICACIÓN CON LA MÁQUINA DE ESTADOS (PickAndPlaceSM)
    # =================================================================
    
    def on_vision_class_received(self, clase: int):
        self.sm.set_object_class(clase)

    def on_vision_target_received(self, target_xyz):
        self.vision_target = target_xyz
        if self.sm.auto_step == 2:
            self.last_detection_time = time.time()
            self.sm.trigger_ctraj_interception(self.vision_target[1])

    def on_vision_dynamic_target_received(self, target_xyz):
        vel_mm_s, _, flag_exitosa = target_xyz
        if self.waiting_for_speed and flag_exitosa == 1.0:
            self.sm.set_calculated_vel(vel_mm_s)
            self.waiting_for_speed = False
            self.tab_dynamic.lbl_dyn_status.setText(f"velocidad tomada exitosamente v={vel_mm_s:.2f} mm/s")
            self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; border-radius: 5px; font-weight: bold;")
            self.tab_dynamic.lbl_vel_val.setText(f"Velocidad Seteada: {vel_mm_s:.2f} mm/s")

    def execute_interception(self):
        if self.estop_active: return
        tracking = self.sm.execute_interception(self.pos_xyz)
        if tracking:
            self.is_tracking_yz = True

    def on_planner_status_received(self, code: int):
        # Delegamos la lógica al archivo separado
        self.sm.process_planner_status(code, self)

    # =================================================================
    # CONSTRUCCIÓN DE LA INTERFAZ GRÁFICA
    # =================================================================

    def init_ui(self):
        self.setWindowTitle("Control y Monitoreo - EEZYbotARM MK2 (STM32 Embedded)")
        self.resize(750, 1020)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tab_control = QWidget()
        self.setup_tab_control()
        self.tabs.addTab(self.tab_control, "🎛️ Control Manual (P2P)")

        self.tab_graphs = QWidget()
        self.setup_tab_graphs()
        self.tabs.addTab(self.tab_graphs, "📈 Gráficos en Tiempo Real")

        self.tab_planner = QWidget()
        self.setup_tab_planner()
        self.tabs.addTab(self.tab_planner, "🤖 Planificador Cartesiano (ctraj)")

        self.tab_vision = QWidget()
        self.setup_tab_vision()
        self.tabs.addTab(self.tab_vision, "👁️ Interfaz de Visión (Estática)")

        self.tab_dynamic = DynamicTab()
        self.tabs.addTab(self.tab_dynamic, "⚡ Interfaz Dinámica (Síncrona)")

        self.tab_dynamic.btn_dyn_set_vel.clicked.connect(self.trigger_set_speed)
        self.tab_dynamic.btn_dyn_exec.clicked.connect(self.execute_interception)
        self.tab_dynamic.btn_dyn_estop.clicked.connect(self.toggle_estop)

        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(33)

    def setup_tab_control(self):
        layout = QVBoxLayout(self.tab_control)

        self.btn_estop = QPushButton("🛑 PARADA DE EMERGENCIA (INACTIVA)")
        self.btn_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_estop.setStyleSheet("background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;")
        self.btn_estop.clicked.connect(self.toggle_estop)
        layout.addWidget(self.btn_estop)

        group_indep = QGroupBox("Acciones Independientes")
        layout_indep = QHBoxLayout()
        self.btn_homing = QPushButton("🏠 EJECUTAR HOMING")
        self.btn_homing.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_homing.setStyleSheet("background-color: #2980b9; color: white; padding: 10px; border-radius: 5px;")
        self.btn_homing.clicked.connect(lambda: self.ros_thread.send_homing())

        self.btn_magnet = QPushButton("🧲 ELECTROIMÁN: APAGADO")
        self.btn_magnet.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_magnet.setStyleSheet("background-color: #7f8c8d; color: white; padding: 10px; border-radius: 5px;")
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
        self.slider_q1.valueChanged.connect(lambda v: self.spin_q1.setValue(float(v)))
        self.spin_q1.valueChanged.connect(lambda v: self.slider_q1.setValue(int(v)))
        grid_sliders.addWidget(self.slider_q1, 0, 1)
        grid_sliders.addWidget(self.spin_q1, 0, 2)

        # LÍMITES AJUSTADOS (Q2 de 40 a 157 para permitir IK en Z bajas)
        grid_sliders.addWidget(QLabel("Q2 [40° a 157°]:"), 1, 0)
        self.slider_q2 = QSlider(Qt.Horizontal)
        self.slider_q2.setRange(40, 157)
        self.slider_q2.setValue(90)
        self.spin_q2 = QDoubleSpinBox()
        self.spin_q2.setRange(40.0, 157.0)
        self.spin_q2.setValue(90.0)
        self.spin_q2.setSingleStep(0.5)
        self.slider_q2.valueChanged.connect(lambda v: self.spin_q2.setValue(float(v)))
        self.spin_q2.valueChanged.connect(lambda v: self.slider_q2.setValue(int(v)))
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
        self.slider_q3.valueChanged.connect(lambda v: self.spin_q3.setValue(float(v)))
        self.spin_q3.valueChanged.connect(lambda v: self.slider_q3.setValue(int(v)))
        grid_sliders.addWidget(self.slider_q3, 2, 1)
        grid_sliders.addWidget(self.spin_q3, 2, 2)

        layout_motors.addLayout(grid_sliders)

        self.btn_move_motors = QPushButton("⚡ ENVIAR ÁNGULOS ARTICULARES (P2P)")
        self.btn_move_motors.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_move_motors.setStyleSheet("background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;")
        self.btn_move_motors.clicked.connect(self.send_joint_cmd)

        layout_motors.addWidget(self.btn_move_motors)
        group_motors.setLayout(layout_motors)
        layout.addWidget(group_motors)

        group_cartesian_ctrl = QGroupBox("Ajuste Cartesiano Directo (P2P / Coordenadas X,Y,Z)")
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

        self.btn_move_cartesian = QPushButton("🎯 ENVIAR CONSIGNA CARTESIANA (IK P2P)")
        self.btn_move_cartesian.setFont(QFont("Arial", 10, QFont.Bold))
        self.btn_move_cartesian.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; border-radius: 5px;")
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

    def setup_tab_graphs(self):
        layout = QVBoxLayout(self.tab_graphs)
        self.canvas = MplCanvas(self, width=5, height=6, dpi=100)
        layout.addWidget(self.canvas)

    def setup_tab_planner(self):
        layout = QVBoxLayout(self.tab_planner)

        self.btn_estop_planner = QPushButton("🛑 PARADA DE EMERGENCIA (INACTIVA)")
        self.btn_estop_planner.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_estop_planner.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; border-radius: 5px;")
        self.btn_estop_planner.clicked.connect(self.toggle_estop)
        layout.addWidget(self.btn_estop_planner)

        group_pstat = QGroupBox("Estado de Ejecución")
        lay_pstat = QVBoxLayout()
        self.lbl_planner_state = QLabel("Estado: ⏸️ REPOSO (IDLE)")
        self.lbl_planner_state.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_planner_state.setAlignment(Qt.AlignCenter)
        self.lbl_planner_state.setStyleSheet("color: #2c3e50; padding: 8px; background-color: #ecf0f1; border-radius: 5px;")
        lay_pstat.addWidget(self.lbl_planner_state)
        group_pstat.setLayout(lay_pstat)
        layout.addWidget(group_pstat)

        group_ctraj = QGroupBox("Planificación Cartesiana Local en STM32 (Polinomio Quíntico C²)")
        lay_ctraj = QVBoxLayout()
        grid_ctraj = QGridLayout()
        grid_ctraj.addWidget(QLabel("Meta X [mm]:"), 0, 0)
        self.spin_cx = QDoubleSpinBox()
        self.spin_cx.setRange(50, 300)
        self.spin_cx.setValue(170)
        grid_ctraj.addWidget(self.spin_cx, 0, 1)

        grid_ctraj.addWidget(QLabel("Meta Y [mm]:"), 0, 2)
        self.spin_cy = QDoubleSpinBox()
        self.spin_cy.setRange(-200, 200)
        grid_ctraj.addWidget(self.spin_cy, 0, 3)

        grid_ctraj.addWidget(QLabel("Meta Z [mm]:"), 0, 4)
        self.spin_cz = QDoubleSpinBox()
        self.spin_cz.setRange(20, 280)
        self.spin_cz.setValue(170)
        grid_ctraj.addWidget(self.spin_cz, 0, 5)

        grid_ctraj.addWidget(QLabel("Duración [s]:"), 1, 0)
        self.spin_cdur = QDoubleSpinBox()
        self.spin_cdur.setRange(0.5, 10.0)
        self.spin_cdur.setValue(1.5)
        self.spin_cdur.setSingleStep(0.2)
        grid_ctraj.addWidget(self.spin_cdur, 1, 1)

        lay_ctraj.addLayout(grid_ctraj)

        btn_ctraj = QPushButton("📐 EJECUTAR CTRAJ QUÍNTICO")
        btn_ctraj.setFont(QFont("Arial", 10, QFont.Bold))
        btn_ctraj.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; border-radius: 5px;")
        btn_ctraj.clicked.connect(lambda: self.ros_thread.send_ctraj_cmd(
            self.spin_cx.value(), self.spin_cy.value(), self.spin_cz.value(), self.spin_cdur.value()
        ))
        lay_ctraj.addWidget(btn_ctraj)
        group_ctraj.setLayout(lay_ctraj)
        layout.addWidget(group_ctraj)
        layout.addStretch(1)

    def setup_tab_vision(self):
        layout = QGridLayout(self.tab_vision)

        vbox_coords = QVBoxLayout()
        group_art = QGroupBox("Coordenadas Articulares")
        layout_art = QVBoxLayout()
        self.lbl_v_q = QLabel("Q1=0.00  Q2=0.00  Q3=0.00")
        self.lbl_v_q.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_v_q.setAlignment(Qt.AlignCenter)
        layout_art.addWidget(self.lbl_v_q)
        group_art.setLayout(layout_art)
        
        group_cart = QGroupBox("Coordenadas Cartesianas")
        layout_cart = QVBoxLayout()
        self.lbl_v_xyz = QLabel("X=0.00  Y=0.00  Z=0.00")
        self.lbl_v_xyz.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_v_xyz.setAlignment(Qt.AlignCenter)
        layout_cart.addWidget(self.lbl_v_xyz)
        group_cart.setLayout(layout_cart)
        
        vbox_coords.addWidget(group_art)
        vbox_coords.addWidget(group_cart)
        layout.addLayout(vbox_coords, 0, 0)

        group_plot = QGroupBox("Grafico Y-Z")
        layout_plot = QVBoxLayout()
        self.canvas_yz = MplCanvas2D_YZ(self, width=4, height=4, dpi=100)
        layout_plot.addWidget(self.canvas_yz)
        group_plot.setLayout(layout_plot)
        layout.addWidget(group_plot, 1, 0)

        group_vision = QGroupBox("Servidor de Vision")
        layout_vision = QVBoxLayout()
        self.lbl_camera = QLabel("Esperando imagen de OpenCV...")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        self.lbl_camera.setStyleSheet("background-color: black; color: white;")
        self.lbl_camera.setMinimumSize(320, 240)
        layout_vision.addWidget(self.lbl_camera)
        group_vision.setLayout(layout_vision)
        layout.addWidget(group_vision, 0, 1)

        vbox_controls = QVBoxLayout()
        self.btn_v_estop = QPushButton("Boton de Parada")
        self.btn_v_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_v_estop.setStyleSheet("background-color: #c0392b; color: white; padding: 12px; border-radius: 5px;")
        self.btn_v_estop.clicked.connect(self.toggle_estop)
        
        self.btn_v_exec = QPushButton("Boton de Ejecucion")
        self.btn_v_exec.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_v_exec.setStyleSheet("background-color: #2980b9; color: white; padding: 12px; border-radius: 5px;")
        self.btn_v_exec.clicked.connect(self.execute_vision_trajectory)
        
        self.lbl_v_status = QLabel("Estado Actual: EN ESPERA")
        self.lbl_v_status.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_v_status.setAlignment(Qt.AlignCenter)
        self.lbl_v_status.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; padding: 12px; border-radius: 5px;")
        
        vbox_controls.addWidget(self.btn_v_estop)
        vbox_controls.addWidget(self.btn_v_exec)
        vbox_controls.addWidget(self.lbl_v_status)
        layout.addLayout(vbox_controls, 1, 1)

    # =================================================================
    # FUNCIONES GENERALES DE INTERFAZ Y ACCIONES P2P
    # =================================================================

    def on_vision_image_received(self, q_img: QImage):
        pixmap = QPixmap.fromImage(q_img)
        self.lbl_camera.setPixmap(pixmap.scaled(self.lbl_camera.size(), Qt.KeepAspectRatio))
        self.tab_dynamic.lbl_camera_dyn.setPixmap(pixmap.scaled(self.tab_dynamic.lbl_camera_dyn.size(), Qt.KeepAspectRatio))

    def trigger_set_speed(self):
        if self.estop_active: return
        self.waiting_for_speed = True
        self.ros_thread.send_trigger_measurement()
        self.tab_dynamic.lbl_dyn_status.setText(
            "⏳ Recalculando velocidad...\nEsperando que el cubo recorra 300 mm."
        )
        self.tab_dynamic.lbl_dyn_status.setStyleSheet(
            "background-color: #f39c12; color: white; padding: 10px; border-radius: 5px;"
        )

    def execute_vision_trajectory(self):
        if self.estop_active: return
        self.vision_start_y = self.pos_xyz[1]
        self.vision_start_z = self.pos_xyz[2]
        self.vision_target_y = self.vision_target[1]
        self.vision_target_z = self.vision_target[2]
        self.yz_trace_y = []
        self.yz_trace_z = []
        self.is_tracking_yz = True
        self.ros_thread.send_ctraj_cmd(self.vision_target[0], self.vision_target[1], self.vision_target[2], duration=1.5)

    def send_joint_cmd(self):
        if self.estop_active: return
        q1_deg = float(self.spin_q1.value())
        q2_deg = float(self.spin_q2.value())
        q3_deg = float(self.spin_q3.value())
        self.q_target = [q1_deg, q2_deg, q3_deg]
        self.ros_thread.send_cmd(q1_deg, q2_deg, q3_deg)

    def send_cartesian_cmd(self):
        if self.estop_active: return
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        success, q_target = self.ros_thread.send_cartesian_cmd(x, y, z)
        if success:
            self.q_target = q_target
            self.spin_q1.setValue(q_target[0])
            self.spin_q2.setValue(q_target[1])
            self.spin_q3.setValue(q_target[2])

    def toggle_magnet(self):
        self.magnet_active = not self.magnet_active
        self.ros_thread.send_magnet(self.magnet_active)
        self.btn_magnet.setText("🧲 ELECTROIMÁN: ENCENDIDO" if self.magnet_active else "🧲 ELECTROIMÁN: APAGADO")

    def toggle_estop(self):
        self.estop_active = not self.estop_active
        self.ros_thread.send_estop(self.estop_active)
        
        # Notificamos a la máquina de estados externa
        self.sm.set_estop(self.estop_active)
        
        # Actualizamos etiquetas en todas las pestañas
        btn_text = "🛑 E-STOP ACTIVADO" if self.estop_active else "🛑 PARADA DE EMERGENCIA (INACTIVA)"
        self.btn_estop.setText(btn_text)
        self.btn_estop_planner.setText(btn_text)
        self.btn_v_estop.setText("E-STOP ACTIVADO" if self.estop_active else "Boton de Parada")
        self.tab_dynamic.btn_dyn_estop.setText(self.btn_v_estop.text())
        
        if self.estop_active:
            self.is_tracking_yz = False

    def on_feedback_received(self, q_real: list, pos_xyz: list):
        self.q_real = q_real
        self.pos_xyz = pos_xyz

        self.lbl_q1_real.setText(f"Q1: {q_real[0]:.2f}°")
        self.lbl_q2_real.setText(f"Q2: {q_real[1]:.2f}°")
        self.lbl_q3_real.setText(f"Q3: {q_real[2]:.2f}°")
        self.lbl_x_real.setText(f"X: {pos_xyz[0]:.2f} mm")
        self.lbl_y_real.setText(f"Y: {pos_xyz[1]:.2f} mm")
        self.lbl_z_real.setText(f"Z: {pos_xyz[2]:.2f} mm")
        
        self.lbl_v_q.setText(f"Q1={q_real[0]:.1f}  Q2={q_real[1]:.1f}  Q3={q_real[2]:.1f}")
        self.lbl_v_xyz.setText(f"X={pos_xyz[0]:.1f}  Y={pos_xyz[1]:.1f}  Z={pos_xyz[2]:.1f}")
        self.tab_dynamic.lbl_dyn_q.setText(self.lbl_v_q.text())
        self.tab_dynamic.lbl_dyn_xyz.setText(self.lbl_v_xyz.text())

        if self.is_tracking_yz:
            self.yz_trace_y.append(pos_xyz[1])
            self.yz_trace_z.append(pos_xyz[2])

    def update_plots(self):
        t_curr = time.time() - self.start_time
        self.time_history.append(t_curr)
        self.q1_real_hist.append(self.q_real[0])
        self.q2_real_hist.append(self.q_real[1])
        self.q3_real_hist.append(self.q_real[2])

        self.q1_target_hist.append(self.q_target[0])
        self.q2_target_hist.append(self.q_target[1])
        self.q3_target_hist.append(self.q_target[2])

        if self.tabs.currentIndex() == 1:
            ax1, ax2, ax3 = self.canvas.axes_q1, self.canvas.axes_q2, self.canvas.axes_q3
            for ax, th, rh, label, col in [(ax1, self.q1_target_hist, self.q1_real_hist, 'Q1 (°)', 'b-'),
                                          (ax2, self.q2_target_hist, self.q2_real_hist, 'Q2 (°)', 'g-'),
                                          (ax3, self.q3_target_hist, self.q3_real_hist, 'Q3 (°)', 'm-')]:
                ax.cla()
                ax.plot(self.time_history, th, 'r--')
                ax.plot(self.time_history, rh, col)
                ax.set_ylabel(label)
                ax.grid(True)
            self.canvas.draw()

        if self.tabs.currentIndex() == 3:
            self.canvas_yz.plot_yz(
                self.vision_start_y, self.vision_start_z,
                self.vision_target_y, self.vision_target_z,
                self.yz_trace_y, self.yz_trace_z
            )


def main(args=None):
    app = QApplication(sys.argv)
    ros_thread = ROS2Thread()
    ros_thread.start()

    window = MainWindow(ros_thread)
    window.show()

    exit_code = app.exec_()
    ros_thread.quit()
    ros_thread.wait()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()