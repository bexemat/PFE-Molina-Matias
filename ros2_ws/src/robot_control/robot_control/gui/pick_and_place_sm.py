# Archivo: robot_control/gui/pick_and_place_sm.py
import time
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread

class PickAndPlaceSM:
    def __init__(self, ros_thread: ROS2Thread, tab_dynamic):
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic
        
        # Máquina de estados:
        # 0: Reposo / E-Stop
        # 1: Moviéndose a Guardia
        # 2: En Guardia | Imán ON | Esperando objeto
        # 3: Intercepción sincronizada en cinta
        # 45: Elevación vertical pura (Lift) y verificación de agarre
        # 5: Traslado elevado hacia caja de clasificación
        # 6: Descenso final en caja de depósito
        # 7: Retorno a Guardia post-descarga o post-aborto
        # 8: Aborto cinemático temporal
        self.auto_step = 0
        self.estop_active = False
        
        # Parámetros geométricos y cinemáticos calibrados
        self.X_CONVEYOR = 215.0        # Centro de la cinta transportadora (mm)
        self.Y_TARGET_PICK = 120.0     # Meta longitudinal de intercepción (mm)
        self.Z_PICK = 78.0             # Altura de contacto magnético (mm)
        self.Z_LIFT = 125.0            # Cota de seguridad de elevación vertical (mm)
        self.T_MIN_ROBOT = 0.85        # Tiempo mínimo viable de trayectoria (s)
        
        self.guard_xyz = [self.X_CONVEYOR, -75.0, 100.0]
        self.current_robot_xyz = list(self.guard_xyz)
        self.POS_TOLERANCE = 2.0       # Tolerancia espacial en lazo cerrado (+/- 2.0 mm)
        
        self.calculated_vel = 0.0
        self.last_seen_y = 0.0
        
        # Clasificación y cerrojo (Latch)
        self.current_obj_class = 1
        self.class_locked = False
        self.current_place_x = 115.0
        self.current_place_y = 115.0
        self.current_place_z = 42.0

    def set_estop(self, state: bool):
        self.estop_active = state
        if state:
            self.auto_step = 0

    def set_robot_pos(self, pos_xyz: list):
        """Actualiza la posición del robot para validar la condición de guardia."""
        self.current_robot_xyz = list(pos_xyz)
        if self.auto_step in [1, 7]:
            if self.is_in_guard_position():
                self.activate_listening_mode()

    def is_in_guard_position(self) -> bool:
        dx = abs(self.current_robot_xyz[0] - self.guard_xyz[0])
        dy = abs(self.current_robot_xyz[1] - self.guard_xyz[1])
        dz = abs(self.current_robot_xyz[2] - self.guard_xyz[2])
        return (dx <= self.POS_TOLERANCE) and (dy <= self.POS_TOLERANCE) and (dz <= self.POS_TOLERANCE)

    def activate_listening_mode(self):
        self.auto_step = 2
        self.class_locked = False
        self.ros_thread.send_magnet(True)
        self.tab_dynamic.lbl_dyn_status.setText("✅ Guardia OK (±2mm) | Imán ON | Esperando objeto...")
        self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; font-weight: bold;")

    def set_calculated_vel(self, vel: float):
        self.calculated_vel = vel

    def set_object_class(self, obj_class: int):
        if not self.class_locked:
            self.current_obj_class = obj_class

    def execute_interception(self, current_pos=None):
        """Inicia el ciclo dinámico posicionando en guardia."""
        if self.estop_active: return False
        
        if self.is_in_guard_position():
            self.activate_listening_mode()
        else:
            self.auto_step = 1
            self.tab_dynamic.lbl_dyn_status.setText("📍 Posicionando en guardia...")
            self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #3498db; color: white; padding: 10px; font-weight: bold;")
            self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=1.2)
        return True

    def trigger_ctraj_interception(self, y_actual: float):
        """Maneja la detección continua, el cerrojo de clase y el disparo de intercepción."""
        if self.estop_active: return
        self.last_seen_y = y_actual

        # Cerrojo de clasificación en el centro óptico
        if -20.0 <= y_actual <= 20.0 and not self.class_locked:
            self.class_locked = True
            clase_txt = "Cubo" if self.current_obj_class == 1 else "Cono"
            print(f"[VISION LOCK] Clase fijada: {clase_txt} en Y={y_actual:.1f} mm.")

        if self.auto_step != 2:
            return

        if self.calculated_vel <= 0:
            return

        distancia_restante = self.Y_TARGET_PICK - y_actual
        if distancia_restante <= 0:
            return

        # Compensación de respuesta electromagnética de 0.15s
        t_calculado = (distancia_restante / self.calculated_vel) - 0.15
        t_objetivo = max(self.T_MIN_ROBOT, t_calculado)

        if t_objetivo >= self.T_MIN_ROBOT:
            self.auto_step = 3
            self.tab_dynamic.lbl_dyn_status.setText(
                f"🎯 INTERCEPTANDO | Y=+{self.Y_TARGET_PICK} mm (T={t_objetivo:.2f}s, V={self.calculated_vel:.1f} mm/s)"
            )
            self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; font-weight: bold;")
            self.ros_thread.send_ctraj_cmd(self.X_CONVEYOR, self.Y_TARGET_PICK, self.Z_PICK, duration=t_objetivo)
        else:
            self.auto_step = 8
            self.class_locked = False
            self.tab_dynamic.lbl_dyn_status.setText("⚠️ Tiempo insuficiente para intercepción.")
            self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #c0392b; color: white; padding: 10px; font-weight: bold;")
            QTimer.singleShot(2000, self.activate_listening_mode)

    def execute_place_sequence(self):
        """Planifica el traslado a la caja asignada según la clase bloqueada."""
        if self.estop_active: return
        self.auto_step = 5
        
        if self.current_obj_class == 1:
            self.current_place_x = 115.0
            self.current_place_y = 115.0
            self.current_place_z = 42.0
            forma_txt = "Cubo"
        else:
            self.current_place_x = 120.0
            self.current_place_y = -120.0
            self.current_place_z = 42.0
            forma_txt = "Cono"

        self.tab_dynamic.lbl_dyn_status.setText(
            f"🚀 Traslado ({forma_txt}) hacia X={self.current_place_x}, Y={self.current_place_y}..."
        )
        self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #2980b9; color: white; padding: 10px; font-weight: bold;")
        # Traslado a altura segura Z=110 mm
        self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, 110.0, duration=1.6)

    def process_planner_status(self, code: int, main_window):
        """Máquina de estados impulsada por eventos del planificador STM32."""
        if code == 1:
            main_window.lbl_planner_state.setText("Estado: ⏳ TRAYECTORIA EN EJECUCIÓN...")
        elif code == 2:
            main_window.lbl_planner_state.setText("Estado: ✅ COMPLETADO")

            if self.auto_step == 1:
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=0.6)

            elif self.auto_step == 3:
                # Paso 1: Elevación vertical pura (Lift)
                self.auto_step = 45
                self.tab_dynamic.lbl_dyn_status.setText("⬆️ Elevando pieza verticalmente (Lift)...")
                self.ros_thread.send_ctraj_cmd(self.X_CONVEYOR, self.Y_TARGET_PICK, self.Z_LIFT, duration=0.6)

            elif self.auto_step == 45:
                # Paso 2: Validación de agarre por visión
                if self.last_seen_y > 135.0:
                    self.auto_step = 7
                    main_window.magnet_active = False
                    self.ros_thread.send_magnet(False)
                    main_window.btn_magnet.setText("🧲 ELECTROIMÁN: APAGADO")
                    self.tab_dynamic.lbl_dyn_status.setText("⚠️ FALLA DE AGARRE: Objeto en cinta. Retornando a guardia...")
                    self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #e74c3c; color: white; padding: 10px; font-weight: bold;")
                    self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=0.9)
                else:
                    self.execute_place_sequence()

            elif self.auto_step == 5:
                # Paso 3: Descenso final a la caja
                self.auto_step = 6
                self.tab_dynamic.lbl_dyn_status.setText(f"⬇️ Descenso final en Z={self.current_place_z} mm...")
                self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, self.current_place_z, duration=0.8)

            elif self.auto_step == 6:
                # Paso 4: Soltar objeto y retornar a guardia
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                main_window.btn_magnet.setText("🧲 ELECTROIMÁN: APAGADO")
                self.tab_dynamic.lbl_dyn_status.setText("🔓 Pieza depositada. Retornando a guardia...")
                self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #9b59b6; color: white; padding: 10px; font-weight: bold;")
                self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=1.4)

            elif self.auto_step == 7:
                # Paso 5: Verificación de guardia y reactivación
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=0.6)

            QTimer.singleShot(50, lambda: setattr(main_window, 'is_tracking_yz', False))