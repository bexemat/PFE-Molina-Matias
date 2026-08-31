# Archivo: robot_control/gui/pick_and_place_sm.py
import time
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread

class PickAndPlaceSM:
    def __init__(self, ros_thread: ROS2Thread, tab_dynamic):
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic  # Para poder actualizar los textos de la UI
        
        # Variables de estado
        self.auto_step = 0
        self.estop_active = False
        self.calculated_vel = 0.0
        
        # Variables de clasificación y destino
        self.current_obj_class = 1
        self.current_place_x = 120.0
        self.current_place_y = 120.0
        self.current_place_z = 42.0

    def set_estop(self, state: bool):
        """Actualiza el estado de emergencia en la máquina de estados."""
        self.estop_active = state
        if state:
            self.auto_step = 0

    def set_calculated_vel(self, vel: float):
        self.calculated_vel = vel

    def set_object_class(self, obj_class: int):
        self.current_obj_class = obj_class

    def check_initial_position(self, current_pos, target_pos, tolerance=2.0):
        return all(abs(current_pos[i] - target_pos[i]) <= tolerance for i in range(3))

    def execute_interception(self, current_pos):
        """Inicia el ciclo (Paso 0 a 1)."""
        if self.estop_active: return
        
        target_espera = [220.0, -75.0, 100.0]
        
        if not self.check_initial_position(current_pos, target_espera, tolerance=2.0):
            self.tab_dynamic.lbl_dyn_status.setText("⚠️ Posición inicial fuera de rango. Corrigiendo...")
            self.auto_step = 0
            self.ros_thread.send_ctraj_cmd(target_espera[0], target_espera[1], target_espera[2], duration=1.0)
            return

        self.auto_step = 1
        self.ros_thread.send_ctraj_cmd(220.0, -75.0, 100.0, duration=1.0)
        self.tab_dynamic.lbl_dyn_status.setText("📍 Posicionando en origen...")
        return True # Indica que debe activar el tracking YZ en la GUI

    def trigger_ctraj_interception(self, y_actual: float):
        """Calcula y ejecuta el MRU (Paso 2 a 3)."""
        if self.auto_step != 2 or self.estop_active: return

        y_meta = 120.0
        distancia_restante = y_meta - y_actual

        if distancia_restante <= 0:
            self.auto_step = 0
            self.tab_dynamic.lbl_dyn_status.setText("⚠️ El cubo rebasó los +120 mm.")
            return

        t_objetivo = distancia_restante / self.calculated_vel
        
        if t_objetivo < 0.1:
            self.auto_step = 0
            self.tab_dynamic.lbl_dyn_status.setText("⚠️ Tiempo insuficiente para llegar a +120 mm.")
            return

        self.auto_step = 3
        self.ros_thread.send_sync_ctraj_cmd(t_objetivo, y_meta, 78.0)
        self.tab_dynamic.lbl_dyn_status.setText(f"🚀 Intercepción hacia Y=+120 mm (T={t_objetivo:.2f}s)...")

    def execute_place_sequence(self):
        """Inicia el movimiento hacia el depósito (Paso 3 a 5)."""
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
            f"🚀 Traslado ({forma_txt}) hacia X={self.current_place_x} Y={self.current_place_y}..."
        )
        self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, 100.0, duration=1.8)

    def process_planner_status(self, code: int, main_window):
        """Maneja la secuencia cuando el planificador reporta estado (Pasos secuenciales)."""
        if code == 1:
            main_window.lbl_planner_state.setText("Estado: ⏳ TRAYECTORIA EN EJECUCIÓN...")
        elif code == 2:
            main_window.lbl_planner_state.setText("Estado: ✅ COMPLETADO")

            if self.auto_step == 0:
                self.auto_step = 1
                self.ros_thread.send_ctraj_cmd(220.0, -75.0, 100.0, duration=1.0)
            
            elif self.auto_step == 1:
                self.auto_step = 2
                main_window.magnet_active = True
                self.ros_thread.send_magnet(True)
                main_window.btn_magnet.setText("🧲 ELECTROIMÁN: ENCENDIDO")
                self.tab_dynamic.lbl_dyn_status.setText("🧲 En espera | Imán ON | Buscando objeto...")
            
            elif self.auto_step == 3:
                self.execute_place_sequence()
            
            elif self.auto_step == 5:
                self.auto_step = 6
                self.tab_dynamic.lbl_dyn_status.setText(f"⬇️ Descenso final en Z={self.current_place_z}...")
                self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, self.current_place_z, duration=0.8)

            elif self.auto_step == 6:
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                main_window.btn_magnet.setText("🧲 ELECTROIMÁN: APAGADO")
                self.tab_dynamic.lbl_dyn_status.setText("🔓 Pieza depositada, retornando al origen...")
                self.ros_thread.send_ctraj_cmd(220.0, -75.0, 100.0, duration=1.5)

            elif self.auto_step == 7:
                self.auto_step = 2
                main_window.magnet_active = True
                self.ros_thread.send_magnet(True)
                main_window.btn_magnet.setText("🧲 ELECTROIMÁN: ENCENDIDO")
                self.tab_dynamic.lbl_dyn_status.setText("🔄 Ciclo reiniciado | Buscando objeto nuevo...")
            
            QTimer.singleShot(50, lambda: setattr(main_window, 'is_tracking_yz', False))