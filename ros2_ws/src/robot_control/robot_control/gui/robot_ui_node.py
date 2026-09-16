"""Nodo supervisor e interfaz gráfica de usuario unificada (GUI V2) para celda robótica.

Este módulo constituye el punto de entrada principal para el control y la supervisión
de la celda de manufactura y clasificación. Integra concurrentemente:
    1. Operación manual y diagnóstico P2P: Modulación directa sobre espacio articular y cartesiano.
    2. Supervisión de visión artificial: Renderizado en tiempo real del flujo de video
       anotado procedente del nodo cenital.
    3. Trazado sagital en tiempo real (Y-Z): Monitoreo gráfico de la trayectoria ejecutada
       por el TCP mediante integración continua a ~30 Hz.
    4. Máquina de estados autónoma: Coordinación de secuencias estáticas y dinámicas continuas
       sincronizadas por eventos con el firmware embebido de la STM32.

Arquitectura de concurrencia:
    Se apoya en la clase ROS2Thread (hilo de trabajo desacoplado) para procesar el ciclo de
    eventos rclpy.spin(), recibiendo telemetría y publicando consignas de forma no bloqueante
    mediante señales Qt seguras (pyqtSignal).
"""

import sys
import math
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
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap, QImage

from robot_control.gui.widgets.mpl_canvas_yz import MplCanvas2D_YZ
from robot_control.gui.ros_thread import ROS2Thread
from robot_control.gui.tabs.dynamic_continuous_tab import (
    DynamicContinuousTab,
)
from robot_control.gui.pick_and_place_sm_v2 import (
    PickAndPlaceSMV2,
)

class MainWindowV2(QMainWindow):
    """Ventana principal de supervisión, control cinemático y monitoreo del robot.

    Administra el ciclo de vida de los componentes gráficos, procesa la telemetría
    articular y cartesiana devuelta por los encoders AS5600, enruta las metas espaciales
    hacia la máquina de estados PickAndPlaceSMV2 y comanda el enclavamiento del E-Stop.

    Atributos:
        ros_thread (ROS2Thread): Instancia del hilo de comunicación reactivo con ROS 2.
        sm (PickAndPlaceSMV2): Instancia del controlador de estados determinista.
        plot_timer (QTimer): Temporizador periódico (~30 Hz) dedicado al renderizado del plano sagital.
    """

    
    def __init__(self, ros_thread: ROS2Thread) -> None:
        """Inicializa componentes visuales y enlaces de comunicación con ROS 2."""
        super().__init__()
        self.ros_thread = ros_thread

        self.estop_active: bool = False
        self.magnet_active: bool = False

        self.q_real: List[float] = [0.0, 90.0, 0.0]
        self.pos_xyz: List[float] = [170.0, 0.0, 170.0]
        self.q_target: List[float] = [0.0, 90.0, 0.0]

        # Parámetros del ciclo estático (Pestaña de Visión Estática)
        self.static_step: int = 0
        self.static_obj_class: int = 1
        self.static_place_x: float = 115.0
        self.static_place_y: float = 115.0
        self.static_place_z: float = 42.0
        self.static_guard_xyz: List[float] = [215.0, 0.0, 150.0]

        # Parámetros de trazado sagital Y-Z
        self.vision_target: List[float] = [0.0, 0.0, 0.0]
        self.vision_start_y: Optional[float] = None
        self.vision_start_z: Optional[float] = None
        self.vision_target_y: Optional[float] = None
        self.vision_target_z: Optional[float] = None
        self.yz_trace_y: List[float] = []
        self.yz_trace_z: List[float] = []
        self.is_tracking_yz: bool = False

        # Conexión de señales provenientes del hilo ROS2Thread
        self.ros_thread.feedback_received.connect(self.on_feedback_received)
        self.ros_thread.planner_status_received.connect(
            self.on_planner_status_received
        )
        self.ros_thread.vision_target_received.connect(
            self.on_vision_target_received
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

        # Temporizador para actualizar el gráfico Y-Z a ~30 Hz
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(33)

    def on_vision_class_received(self, clase: int) -> None:
        self.sm.set_object_class(clase)
        self.static_obj_class = clase

    def on_vision_target_received(self, target_xyz: List[float]) -> None:
        """Recibe la posición estática del objeto detectado por visión."""
        self.vision_target = target_xyz

    def on_vision_dynamic_target_received(
        self, target_xyz: List[float]
    ) -> None:
        """Recibe [vel_mm_s, y_actual, flag_medido] del nodo de tracking de visión."""
        vel_mm_s, y_actual, flag_medido = target_xyz

        self.tab_dynamic_v2.lbl_dyn_vision.setText(
            f"Y: {y_actual:.1f} mm "
        )

        if self.sm.is_active():
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

    def execute_vision_trajectory(self) -> None:
        """Inicia la secuencia de pick-and-place estático con movimientos suaves."""
        if self.estop_active:
            return

        self.magnet_active = True
        self.ros_thread.send_magnet(True)
        self.btn_magnet.setText("ELECTROIMÁN: ENCENDIDO")

        if self.static_obj_class == 1:
            self.static_place_x = 115.0
            self.static_place_y = 115.0
            self.static_place_z = 42.0
        else:
            self.static_place_x = 120.0
            self.static_place_y = -120.0
            self.static_place_z = 42.0

        self.vision_start_y = self.pos_xyz[1]
        self.vision_start_z = self.pos_xyz[2]
        self.vision_target_y = self.vision_target[1]
        self.vision_target_z = self.vision_target[2]
        self.yz_trace_y = []
        self.yz_trace_z = []
        self.is_tracking_yz = True

        self.static_step = 1
        self.lbl_v_status.setText("Estado: PICK hacia objeto...")
        self.lbl_v_status.setStyleSheet(
            "background-color: #8e44ad; color: white; padding: 12px; border-radius: 5px; font-weight: bold;"
        )
        self.ros_thread.send_ctraj_cmd(
            self.vision_target[0],
            self.vision_target[1],
            self.vision_target[2],
            duration=1.0,
        )

    def on_planner_status_received(self, code: int) -> None:
        """Procesa códigos de estado publicados por el firmware en /planner/traj_status."""
        if self.sm.is_active():
            self.sm.process_planner_status(code, self)
            return

        # Lógica de la secuencia estática con transiciones suaves idénticas al modo dinámico
        if code == 1:
            if hasattr(self, "lbl_v_status"):
                self.lbl_v_status.setText("Estado: TRAYECTORIA EN EJECUCIÓN...")
        elif code in [2, 3]:  # Acepta SUCCESS (2) y Asentamiento/TIMEOUT (3) del firmware
            if hasattr(self, "lbl_v_status"):
                self.lbl_v_status.setText("Estado: COMPLETADO")

            if self.static_step == 1:
                # Fase 1: Elevación vertical pura (Lift) suave
                self.static_step = 2
                target_lift = [
                    self.vision_target[0],
                    self.vision_target[1],
                    125.0,
                ]
                self.lbl_v_status.setText("Estado: Elevando pieza (Lift)...")
                self.lbl_v_status.setStyleSheet(
                    "background-color: #2980b9; color: white; padding: 12px; border-radius: 5px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    target_lift[0], target_lift[1], target_lift[2], duration=0.6
                )

            elif self.static_step == 2:
                # Fase 2: Traslado elevado hacia el contenedor (Suave y parsimonioso)
                self.static_step = 3
                target_box_high = [
                    self.static_place_x,
                    self.static_place_y,
                    110.0,
                ]
                clase_str = "Cubo" if self.static_obj_class == 1 else "Cono"
                self.lbl_v_status.setText(
                    f"Estado: PLACE ({clase_str}) hacia X={self.static_place_x}, Y={self.static_place_y}..."
                )
                self.lbl_v_status.setStyleSheet(
                    "background-color: #3498db; color: white; padding: 12px; border-radius: 5px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    target_box_high[0],
                    target_box_high[1],
                    target_box_high[2],
                    duration=1.6,
                )

            elif self.static_step == 3:
                # Fase 3: Descenso final controlado en el contenedor
                self.static_step = 4
                target_box_low = [
                    self.static_place_x,
                    self.static_place_y,
                    self.static_place_z,
                ]
                self.lbl_v_status.setText(
                    f"Estado: Descendiendo a Z={self.static_place_z} mm..."
                )
                self.ros_thread.send_ctraj_cmd(
                    target_box_low[0],
                    target_box_low[1],
                    target_box_low[2],
                    duration=0.8,
                )

            elif self.static_step == 4:
                # Fase 4: Soltar pieza y retorno suave a guardia
                self.static_step = 5
                self.magnet_active = False
                self.ros_thread.send_magnet(False)
                self.btn_magnet.setText("ELECTROIMÁN: APAGADO")

                self.lbl_v_status.setText(
                    "Estado: Pieza depositada. Retornando a guardia..."
                )
                self.lbl_v_status.setStyleSheet(
                    "background-color: #27ae60; color: white; padding: 12px; border-radius: 5px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    self.static_guard_xyz[0],
                    self.static_guard_xyz[1],
                    self.static_guard_xyz[2],
                    duration=1.4,
                )

            elif self.static_step == 5:
                self.static_step = 0
                self.is_tracking_yz = False
                self.lbl_v_status.setText("Estado Actual: CICLO COMPLETADO")
                self.lbl_v_status.setStyleSheet(
                    "background-color: #ecf0f1; color: #2c3e50; padding: 12px; border-radius: 5px; font-weight: bold;"
                )
        elif code < 0:
            print(f"[Aviso P2P] Código de trayectoria recibido del firmware: {code}")

    def update_plots(self) -> None:
        """Actualiza el gráfico Y-Z en tiempo real, gestiona el trazado y fuerza el renderizado."""
        if self.sm.auto_step == 2:
            self.yz_trace_y.clear()
            self.yz_trace_z.clear()
            self.is_tracking_yz = False

        if self.sm.is_active() and self.sm.auto_step >= 4:
            if not self.is_tracking_yz:
                self.vision_start_y = self.pos_xyz[1]
                self.vision_start_z = self.pos_xyz[2]
                self.vision_target_y = self.sm.Y_TARGET_PICK
                self.vision_target_z = self.sm.Z_PICK
                self.yz_trace_y = []
                self.yz_trace_z = []
                self.is_tracking_yz = True

        if self.is_tracking_yz:
            self.yz_trace_y.append(self.pos_xyz[1])
            self.yz_trace_z.append(self.pos_xyz[2])

        # Renderizado en la pestaña dinámica
        canvas = self.tab_dynamic_v2.canvas_yz
        canvas.plot_yz(
            self.vision_start_y,
            self.vision_start_z,
            self.vision_target_y,
            self.vision_target_z,
            self.yz_trace_y,
            self.yz_trace_z,
        )
        if hasattr(canvas, "fig"):
            canvas.fig.tight_layout()
        canvas.draw()

        # Renderizado en la pestaña de visión estática
        if hasattr(self, "canvas_yz_static"):
            self.canvas_yz_static.plot_yz(
                self.vision_start_y,
                self.vision_start_z,
                self.vision_target_y,
                self.vision_target_z,
                self.yz_trace_y,
                self.yz_trace_z,
            )
            if hasattr(self.canvas_yz_static, "fig"):
                self.canvas_yz_static.fig.tight_layout()
            self.canvas_yz_static.draw()

    def init_ui(self) -> None:
        """Construye las pestañas de control manual, visión estática y dinámico continuo."""
        self.setWindowTitle(
            "Control y Monitoreo Celda Robotizada"
        )
        self.resize(850, 980)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tab_control = QWidget()
        self.setup_tab_control()
        self.tabs.addTab(self.tab_control, "Control Manual (P2P)")

        self.tab_vision = QWidget()
        self.setup_tab_vision()
        self.tabs.addTab(self.tab_vision, "Modo Estático Continuo)")

        self.tab_dynamic_v2 = DynamicContinuousTab()
        self.tabs.addTab(self.tab_dynamic_v2, "Modo Dinámico Continuo")
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

        grid_sliders.addWidget(QLabel("Q2 [20° a 157°]:"), 1, 0)
        self.slider_q2 = QSlider(Qt.Horizontal)
        self.slider_q2.setRange(20, 157)
        self.slider_q2.setValue(90)
        self.spin_q2 = QDoubleSpinBox()
        self.spin_q2.setRange(20.0, 157.0)
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

    def setup_tab_vision(self) -> None:
        """Configura los componentes visuales de la pestaña de Visión Estática."""
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

        group_plot = QGroupBox("Gráfico Y-Z (Estático)")
        layout_plot = QVBoxLayout()
        self.canvas_yz_static = MplCanvas2D_YZ(self, width=4, height=4, dpi=100)
        layout_plot.addWidget(self.canvas_yz_static)
        group_plot.setLayout(layout_plot)
        layout.addWidget(group_plot, 1, 0)

        group_vision = QGroupBox("Servidor de Visión")
        layout_vision = QVBoxLayout()
        self.lbl_camera = QLabel("Esperando imagen de OpenCV...")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        self.lbl_camera.setStyleSheet("background-color: black; color: white;")
        self.lbl_camera.setMinimumSize(320, 240)
        layout_vision.addWidget(self.lbl_camera)
        group_vision.setLayout(layout_vision)
        layout.addWidget(group_vision, 0, 1)

        vbox_controls = QVBoxLayout()
        self.btn_v_estop = QPushButton("Botón de Parada")
        self.btn_v_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_v_estop.setStyleSheet(
            "background-color: #c0392b; color: white; padding: 12px; border-radius: 5px;"
        )
        self.btn_v_estop.clicked.connect(self.toggle_estop)

        self.btn_v_exec = QPushButton("Botón de Ejecución (Estático)")
        self.btn_v_exec.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_v_exec.setStyleSheet(
            "background-color: #2980b9; color: white; padding: 12px; border-radius: 5px;"
        )
        self.btn_v_exec.clicked.connect(self.execute_vision_trajectory)

        self.lbl_v_status = QLabel("Estado Actual: EN ESPERA")
        self.lbl_v_status.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_v_status.setAlignment(Qt.AlignCenter)
        self.lbl_v_status.setStyleSheet(
            "background-color: #ecf0f1; color: #2c3e50; padding: 12px; border-radius: 5px;"
        )

        vbox_controls.addWidget(self.btn_v_estop)
        vbox_controls.addWidget(self.btn_v_exec)
        vbox_controls.addWidget(self.lbl_v_status)
        layout.addLayout(vbox_controls, 1, 1)

    def on_vision_image_received(self, q_img: QImage) -> None:
        """Actualiza el flujo de video en las vistas de ambas pestañas."""
        if q_img.isNull():
            return
        pixmap = QPixmap.fromImage(q_img)
        
        # Visión Estática
        if hasattr(self, "lbl_camera"):
            self.lbl_camera.setPixmap(
                pixmap.scaled(self.lbl_camera.size(), Qt.KeepAspectRatio)
            )

        # Dinámico Continuo
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

        if self.estop_active:
            self.static_step = 0
            self.is_tracking_yz = False

        btn_text = (
            "PARADA DE EMERGENCIA ACTIVADA"
            if self.estop_active
            else "PARADA DE EMERGENCIA (INACTIVA)"
        )
        self.btn_estop.setText(btn_text)
        self.tab_dynamic_v2.btn_dyn_estop.setText(btn_text)
        if hasattr(self, "btn_v_estop"):
            self.btn_v_estop.setText(
                "E-STOP ACTIVADO" if self.estop_active else "Botón de Parada"
            )

    def on_feedback_received(
        self, q_real: List[float], pos_xyz: List[float]
    ) -> None:
        self.q_real = q_real
        self.pos_xyz = pos_xyz
        
        self.sm.set_robot_telemetry(q_real, pos_xyz)

        # Tab Manual (P2P)
        self.lbl_q1_real.setText(f"Q1: {q_real[0]:.2f}°")
        self.lbl_q2_real.setText(f"Q2: {q_real[1]:.2f}°")
        self.lbl_q3_real.setText(f"Q3: {q_real[2]:.2f}°")
        self.lbl_x_real.setText(f"X: {pos_xyz[0]:.2f} mm")
        self.lbl_y_real.setText(f"Y: {pos_xyz[1]:.2f} mm")
        self.lbl_z_real.setText(f"Z: {pos_xyz[2]:.2f} mm")

        # Tab Visión Estática
        if hasattr(self, "lbl_v_q") and hasattr(self, "lbl_v_xyz"):
            self.lbl_v_q.setText(
                f"Q1={q_real[0]:.1f}  Q2={q_real[1]:.1f}  Q3={q_real[2]:.1f}"
            )
            self.lbl_v_xyz.setText(
                f"X={pos_xyz[0]:.1f}  Y={pos_xyz[1]:.1f}  Z={pos_xyz[2]:.1f}"
            )

        # Tab Dinámico Continuo
        v_q_txt = f"Q1={q_real[0]:.1f}°  Q2={q_real[1]:.1f}°  Q3={q_real[2]:.1f}°"
        self.tab_dynamic_v2.lbl_dyn_q.setText(v_q_txt)

        fk_txt = f"X: {pos_xyz[0]:.1f} | Y: {pos_xyz[1]:.1f} | Z: {pos_xyz[2]:.1f}"
        self.tab_dynamic_v2.lbl_dyn_fk.setText(fk_txt)


def main(args: Optional[list] = None) -> None:
    """Punto de arranque del proceso gráfico y enlace con el entorno ROS 2.

    Inicializa el bucle de eventos de QApplication, dispara el hilo trabajador
    ROS2Thread en segundo plano, despliega la ventana MainWindowV2 y asegura
    el cierre ordenado y sincrónico de ambos entornos al finalizar la ejecución.

    Args:
        args (Optional[list]): Argumentos opcionales de línea de comandos para la aplicación.
    """
   
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