"""Pestaña de control visual para el modo dinámico continuo en 1 paso."""

import os
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPushButton,
    QLabel,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QDoubleSpinBox,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPixmap

from robot_control.gui.widgets.mpl_canvas_yz import MplCanvas2D_YZ


class DynamicContinuousTab(QWidget):
    """Pestaña orientada a la operación autónoma continua sin calibraciones intermedias."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self) -> None:
        """Construye controles de guardia, telemetría y visualización de cámara."""
        layout = QGridLayout(self)
        font_bold_data = QFont("Arial", 11, QFont.Bold)

        vbox_coords = QVBoxLayout()

        # 1. Coordenadas Articulares
        group_art = QGroupBox("Coordenadas Articulares (FK Sensors)")
        layout_art = QVBoxLayout()
        self.lbl_dyn_q = QLabel("Q1=0.00°  Q2=90.00°  Q3=0.00°")
        self.lbl_dyn_q.setFont(font_bold_data)
        self.lbl_dyn_q.setAlignment(Qt.AlignCenter)
        layout_art.addWidget(self.lbl_dyn_q)
        group_art.setLayout(layout_art)

        # 2. Coordenadas Cartesianas (FK vs Visión)
        group_cart = QGroupBox("Monitoreo Cartesiano [mm]")
        grid_cart = QGridLayout()

        # Fila FK
        lbl_title_fk = QLabel("FK Robot (TCP):")
        lbl_title_fk.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_dyn_fk = QLabel("X: 0.00 | Y: 0.00 | Z: 0.00")
        self.lbl_dyn_fk.setFont(font_bold_data)
        self.lbl_dyn_fk.setStyleSheet("color: #27ae60;")
        grid_cart.addWidget(lbl_title_fk, 0, 0)
        grid_cart.addWidget(self.lbl_dyn_fk, 0, 1)

        # Fila Visión
        lbl_title_vis = QLabel("Medido Visión:")
        lbl_title_vis.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_dyn_vision = QLabel("Y: -- mm ")
        self.lbl_dyn_vision.setFont(font_bold_data)
        self.lbl_dyn_vision.setStyleSheet("color: #2980b9;")
        grid_cart.addWidget(lbl_title_vis, 1, 0)
        grid_cart.addWidget(self.lbl_dyn_vision, 1, 1)

        # Compatibilidad hacia atrás
        self.lbl_dyn_xyz = self.lbl_dyn_fk

        group_cart.setLayout(grid_cart)

        # 3. Rectángulo y cartel de Estado de Clasificación
        self.group_class_status = QGroupBox("Estado Actual de Clasificación")
        lay_class = QVBoxLayout()
        self.lbl_class_status = QLabel("Clasificación: INACTIVO")
        self.lbl_class_status.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_class_status.setAlignment(Qt.AlignCenter)
        self.lbl_class_status.setStyleSheet(
            "background-color: #bdc3c7; color: #2c3e50; padding: 10px; border-radius: 5px;"
        )
        lay_class.addWidget(self.lbl_class_status)
        self.group_class_status.setLayout(lay_class)
        self.group_class_status.setVisible(False)

        # 4. Gráfico Y-Z (Dimensiones ampliadas)
        group_plot_yz = QGroupBox("Gráfico Y-Z")
        layout_plot_yz = QVBoxLayout()
        self.canvas_yz = MplCanvas2D_YZ(self, width=5, height=4, dpi=100)
        layout_plot_yz.addWidget(self.canvas_yz)
        group_plot_yz.setLayout(layout_plot_yz)

        # 5. Margen Inferior Izquierdo: Banner institucional libre (sin recuadros)
        widget_institutional = QWidget()
        lay_inst = QHBoxLayout()
        lay_inst.setContentsMargins(0, 5, 0, 0)
        
        self.lbl_logo = QLabel()
        self.lbl_logo.setAlignment(Qt.AlignCenter)
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        logo_path = os.path.join(current_dir, "Imagen", "LogoNombre.png")
        
        pixmap_logo = QPixmap(logo_path)
        if not pixmap_logo.isNull():
            self.lbl_logo.setPixmap(
                pixmap_logo.scaled(720, 220, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        else:
            self.lbl_logo.setText("[ Banner Institucional ]")
            
        lay_inst.addWidget(self.lbl_logo)
        widget_institutional.setLayout(lay_inst)

        vbox_coords.addWidget(group_art)
        vbox_coords.addWidget(group_cart)
        vbox_coords.addWidget(self.group_class_status)
        vbox_coords.addWidget(group_plot_yz)
        vbox_coords.addWidget(widget_institutional)
        
        layout.addLayout(vbox_coords, 0, 0)

        # 6. Flujo de Visión Artificial
        group_vision_dyn = QGroupBox("Servidor de Visión (Tracking Continuo)")
        layout_vision_dyn = QVBoxLayout()
        self.lbl_camera_dyn = QLabel("Esperando flujo continuo de video...")
        self.lbl_camera_dyn.setAlignment(Qt.AlignCenter)
        self.lbl_camera_dyn.setStyleSheet("background-color: black; color: white;")
        self.lbl_camera_dyn.setMinimumSize(320, 240)
        self.lbl_camera_dyn.setScaledContents(True)
        layout_vision_dyn.addWidget(self.lbl_camera_dyn)
        group_vision_dyn.setLayout(layout_vision_dyn)
        layout.addWidget(group_vision_dyn, 0, 1)

        # 7. Parámetros de Operación Dinámica
        vbox_controls_dyn = QVBoxLayout()

        self.btn_dyn_estop = QPushButton("Botón de Parada")
        self.btn_dyn_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_estop.setStyleSheet(
            "background-color: #c0392b; color: white; padding: 10px; border-radius: 5px;"
        )

        group_guard = QGroupBox("Posición Inicial de Guardia")
        lay_guard = QHBoxLayout()

        lay_guard.addWidget(QLabel("X [mm]:"))
        self.spin_gx = QDoubleSpinBox()
        self.spin_gx.setRange(50.0, 300.0)
        self.spin_gx.setValue(215.0)
        lay_guard.addWidget(self.spin_gx)

        lay_guard.addWidget(QLabel("Y [mm]:"))
        self.spin_gy = QDoubleSpinBox()
        self.spin_gy.setRange(-150.0, 150.0)
        self.spin_gy.setValue(-75.0)
        lay_guard.addWidget(self.spin_gy)

        lay_guard.addWidget(QLabel("Z [mm]:"))
        self.spin_gz = QDoubleSpinBox()
        self.spin_gz.setRange(40.0, 250.0)
        self.spin_gz.setValue(100.0)
        lay_guard.addWidget(self.spin_gz)
        group_guard.setLayout(lay_guard)

        self.lbl_vel_val = QLabel("Velocidad Estimada: 0.00 mm/s")
        self.lbl_vel_val.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_vel_val.setAlignment(Qt.AlignCenter)
        self.lbl_vel_val.setStyleSheet(
            "background-color: #34495e; color: #ecf0f1; padding: 8px; border-radius: 4px;"
        )

        self.btn_dyn_start = QPushButton("INICIAR MODO DINÁMICO CONTINUO")
        self.btn_dyn_start.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_start.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;"
        )

        self.lbl_dyn_status = QLabel("Estado: EN ESPERA")
        self.lbl_dyn_status.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_dyn_status.setAlignment(Qt.AlignCenter)
        self.lbl_dyn_status.setStyleSheet(
            "background-color: #ecf0f1; color: #2c3e50; padding: 10px; border-radius: 5px;"
        )

        vbox_controls_dyn.addWidget(self.btn_dyn_estop)
        vbox_controls_dyn.addWidget(group_guard)
        vbox_controls_dyn.addWidget(self.lbl_vel_val)
        vbox_controls_dyn.addWidget(self.btn_dyn_start)
        vbox_controls_dyn.addWidget(self.lbl_dyn_status)
        
        layout.addLayout(vbox_controls_dyn, 1, 1)