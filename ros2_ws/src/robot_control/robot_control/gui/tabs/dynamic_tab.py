"""Pestaña visual para la operación dinámica síncrona en cinta transportadora."""

from typing import Optional
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPushButton,
    QLabel,
    QGroupBox,
    QGridLayout,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class DynamicTab(QWidget):
    """Pestaña dedicada a la intercepción síncrona en movimiento (Y_pick = +120 mm)."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self) -> None:
        """Construye los layouts, etiquetas y pulsadores de la pestaña."""
        layout = QGridLayout(self)

        vbox_coords = QVBoxLayout()
        group_art = QGroupBox("Coordenadas Articulares (Dinámico)")
        layout_art = QVBoxLayout()
        self.lbl_dyn_q = QLabel("Q1=0.00  Q2=0.00  Q3=0.00")
        self.lbl_dyn_q.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_dyn_q.setAlignment(Qt.AlignCenter)
        layout_art.addWidget(self.lbl_dyn_q)
        group_art.setLayout(layout_art)

        group_cart = QGroupBox("Coordenadas Cartesianas (Dinámico)")
        layout_cart = QVBoxLayout()
        self.lbl_dyn_xyz = QLabel("X=0.00  Y=0.00  Z=0.00")
        self.lbl_dyn_xyz.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_dyn_xyz.setAlignment(Qt.AlignCenter)
        layout_cart.addWidget(self.lbl_dyn_xyz)
        group_cart.setLayout(layout_cart)

        vbox_coords.addWidget(group_art)
        vbox_coords.addWidget(group_cart)
        layout.addLayout(vbox_coords, 0, 0)

        group_vision_dyn = QGroupBox("Servidor de Visión (Cinta en Movimiento)")
        layout_vision_dyn = QVBoxLayout()
        self.lbl_camera_dyn = QLabel("Esperando flujo dinámico de video...")
        self.lbl_camera_dyn.setAlignment(Qt.AlignCenter)
        self.lbl_camera_dyn.setStyleSheet("background-color: black; color: white;")
        self.lbl_camera_dyn.setMinimumSize(320, 240)
        layout_vision_dyn.addWidget(self.lbl_camera_dyn)
        group_vision_dyn.setLayout(layout_vision_dyn)
        layout.addWidget(group_vision_dyn, 0, 1)

        vbox_controls_dyn = QVBoxLayout()

        self.btn_dyn_estop = QPushButton("Boton de Parada")
        self.btn_dyn_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_estop.setStyleSheet(
            "background-color: #c0392b; color: white; padding: 10px; border-radius: 5px;"
        )

        self.btn_dyn_set_vel = QPushButton(
            "1. Setear / Calcular Velocidad (300 mm)"
        )
        self.btn_dyn_set_vel.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_set_vel.setStyleSheet(
            "background-color: #d35400; color: white; padding: 10px; border-radius: 5px;"
        )

        self.lbl_vel_val = QLabel("Velocidad Seteada: 0.00 mm/s")
        self.lbl_vel_val.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_vel_val.setAlignment(Qt.AlignCenter)
        self.lbl_vel_val.setStyleSheet(
            "background-color: #34495e; color: #ecf0f1; padding: 8px; border-radius: 4px;"
        )

        self.btn_dyn_exec = QPushButton(
            "2. Calcular Tiempo e Interceptar (+120 mm)"
        )
        self.btn_dyn_exec.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_exec.setStyleSheet(
            "background-color: #2980b9; color: white; padding: 10px; border-radius: 5px;"
        )

        self.lbl_dyn_status = QLabel("Estado: Esperando calibración de velocidad...")
        self.lbl_dyn_status.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_dyn_status.setAlignment(Qt.AlignCenter)
        self.lbl_dyn_status.setStyleSheet(
            "background-color: #ecf0f1; color: #2c3e50; padding: 10px; border-radius: 5px;"
        )

        vbox_controls_dyn.addWidget(self.btn_dyn_estop)
        vbox_controls_dyn.addWidget(self.btn_dyn_set_vel)
        vbox_controls_dyn.addWidget(self.lbl_vel_val)
        vbox_controls_dyn.addWidget(self.btn_dyn_exec)
        vbox_controls_dyn.addWidget(self.lbl_dyn_status)
        layout.addLayout(vbox_controls_dyn, 1, 1)