# Archivo: robot_control/gui/tabs/dynamic_continuous_tab.py
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QLabel, QGroupBox, 
    QGridLayout, QHBoxLayout, QDoubleSpinBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class DynamicContinuousTab(QWidget):
    """Pestaña dedicada a la Interfaz Dinámica Continua en 1 Paso."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self):
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

        group_vision_dyn = QGroupBox("Servidor de Visión (Tracking Continuo)")
        layout_vision_dyn = QVBoxLayout()
        self.lbl_camera_dyn = QLabel("Esperando flujo continuo...")
        self.lbl_camera_dyn.setAlignment(Qt.AlignCenter)
        self.lbl_camera_dyn.setStyleSheet("background-color: black; color: white;")
        self.lbl_camera_dyn.setMinimumSize(320, 240)
        self.lbl_camera_dyn.setScaledContents(True)
        layout_vision_dyn.addWidget(self.lbl_camera_dyn)
        group_vision_dyn.setLayout(layout_vision_dyn)
        layout.addWidget(group_vision_dyn, 0, 1)

        vbox_controls_dyn = QVBoxLayout()
        
        # Botón de Parada de Emergencia
        self.btn_dyn_estop = QPushButton("🛑 Botón de Parada")
        self.btn_dyn_estop.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_estop.setStyleSheet("background-color: #c0392b; color: white; padding: 10px; border-radius: 5px;")

        # Configuración de Posición Inicial de Guardia
        group_guard = QGroupBox("Posición Inicial de Guardia")
        lay_guard = QHBoxLayout()
        
        # En dynamic_continuous_tab.py -> setup_ui()
        lay_guard.addWidget(QLabel("X:"))
        self.spin_gx = QDoubleSpinBox()
        self.spin_gx.setRange(50.0, 300.0)
        self.spin_gx.setValue(215.0)  # <-- Valor centrado en la cinta
        lay_guard.addWidget(self.spin_gx)

        lay_guard.addWidget(QLabel("Y:"))
        self.spin_gy = QDoubleSpinBox()
        self.spin_gy.setRange(-150.0, 150.0)
        self.spin_gy.setValue(-75.0)
        lay_guard.addWidget(self.spin_gy)

        lay_guard.addWidget(QLabel("Z:"))
        self.spin_gz = QDoubleSpinBox()
        self.spin_gz.setRange(40.0, 250.0)
        self.spin_gz.setValue(100.0)
        lay_guard.addWidget(self.spin_gz)
        group_guard.setLayout(lay_guard)

        # Visualizador de Telemetría Dinámica
        self.lbl_vel_val = QLabel("Velocidad Estimada: 0.00 mm/s")
        self.lbl_vel_val.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_vel_val.setAlignment(Qt.AlignCenter)
        self.lbl_vel_val.setStyleSheet("background-color: #34495e; color: #ecf0f1; padding: 8px; border-radius: 4px;")

        # Botón Único
        self.btn_dyn_start = QPushButton("🚀 INICIAR MODO DINÁMICO CONTINUO")
        self.btn_dyn_start.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_dyn_start.setStyleSheet("background-color: #27ae60; color: white; padding: 12px; border-radius: 5px;")
        
        self.lbl_dyn_status = QLabel("Estado: EN ESPERA")
        self.lbl_dyn_status.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_dyn_status.setAlignment(Qt.AlignCenter)
        self.lbl_dyn_status.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; padding: 10px; border-radius: 5px;")
        
        vbox_controls_dyn.addWidget(self.btn_dyn_estop)
        vbox_controls_dyn.addWidget(group_guard)
        vbox_controls_dyn.addWidget(self.lbl_vel_val)
        vbox_controls_dyn.addWidget(self.btn_dyn_start)
        vbox_controls_dyn.addWidget(self.lbl_dyn_status)
        layout.addLayout(vbox_controls_dyn, 1, 1)