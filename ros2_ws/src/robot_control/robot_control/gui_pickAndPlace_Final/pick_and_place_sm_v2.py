# Archivo: robot_control/gui_pickAndPlace_Final/pick_and_place_sm_v2.py
import time
import numpy as np
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread
from robot_control.gui.kinematics import inverse_kinematics

class PickAndPlaceSMV2:
    """
    Máquina de Estados para Operaciones Continuas y Dinámicas de Pick-and-Place (V2).
    
    Nota de Arquitectura (Ingeniería de Software Senior):
    Esta clase desacopla la lógica operativa de alto nivel de la GUI y de las capas 
    de comunicación de ROS 2. Implementa una máquina de estados determinista para 
    gestionar la intercepción en la cinta transportadora, incorporando telemetría de 
    visión en tiempo real, tolerancias espaciales estrictas en lazo cerrado y 
    recuperación de fallos (ej. verificación continua de agarre).
    """
    def __init__(self, ros_thread: ROS2Thread, tab_dynamic):
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic
        
        # Definición de la Máquina de Estados:
        # 0: IDLE / Sistema Detenido
        # 1: Viajando a la Posición de Guardia Inicial (Verificación de tolerancia en lazo cerrado)
        # 2: En Guardia | Imán ON | Esperando ingreso del objeto
        # 4: Trayectoria Quíntica de Intercepción (Y=+120mm) activa
        # 45: Fase de Elevación Vertical Pura post-agarre para prevenir fuerzas de corte
        # 5: Traslado elevado hacia la caja de clasificación asignada
        # 6: Descenso final en la caja de depósito
        # 7: Retorno a Guardia post-descarga o post-aborto
        # 8: Estado de aborto cinemático temporal (espera de seguridad y reseteo)
        self.auto_step = 0
        self.estop_active = False
        
        # Constantes Operativas (Calibradas al hardware físico)
        self.X_CONVEYOR = 215.0        # Centro exacto de la cinta transportadora (mm)
        self.Y_TARGET_PICK = 120.0     # Meta longitudinal de intercepción (mm)
        self.Z_PICK = 78.0             # Altura de contacto magnético firme (mm)
        self.Z_LIFT = 125.0            # Altura de seguridad para la elevación vertical (mm)
        self.T_MIN_ROBOT = 0.85        # Tiempo mínimo viable de ejecución (s) para prevenir pérdida de pasos
        
        self.current_robot_xyz = [self.X_CONVEYOR, -75.0, 100.0]
        self.guard_xyz = [self.X_CONVEYOR, -75.0, 100.0]
        self.POS_TOLERANCE = 2.0       # Tolerancia estricta en lazo cerrado (+/- 2.0 mm)
        
        self.calculated_vel = 0.0
        self.last_seen_y = 0.0
        
        # Configuración de Clasificación y Depósito
        self.current_obj_class = 1
        self.class_locked = False      # Mecanismo de cerrojo (latch) para prevenir errores por oclusión visual
        self.current_place_x = 115.0
        self.current_place_y = 115.0
        self.current_place_z = 42.0

    def set_estop(self, state: bool):
        """Sobrescritura de la Parada de Emergencia (E-Stop) a nivel de hardware."""
        self.estop_active = state
        if state:
            self.auto_step = 0

    def set_robot_pos(self, pos_xyz: list):
        """Callback para actualizar la retroalimentación cinemática del robot y validar el bucle de guardia."""
        self.current_robot_xyz = list(pos_xyz)
        if self.auto_step in [1, 7]:
            if self.is_in_guard_position():
                self.activate_listening_mode()

    def is_in_guard_position(self) -> bool:
        """Valida si el efector final se encuentra dentro de la estricta matriz de tolerancia espacial."""
        dx = abs(self.current_robot_xyz[0] - self.guard_xyz[0])
        dy = abs(self.current_robot_xyz[1] - self.guard_xyz[1])
        dz = abs(self.current_robot_xyz[2] - self.guard_xyz[2])
        return (dx <= self.POS_TOLERANCE) and (dy <= self.POS_TOLERANCE) and (dz <= self.POS_TOLERANCE)

    def activate_listening_mode(self):
        """Transiciona el sistema al estado de escucha activa, listo para nuevos objetivos dinámicos."""
        self.auto_step = 2
        self.class_locked = False      # Liberar el cerrojo para el nuevo ciclo
        self.ros_thread.send_magnet(True)
        self.tab_dynamic.lbl_dyn_status.setText("✅ Guardia OK (±2mm) | Imán ON | Esperando objeto...")
        self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; font-weight: bold;")

    def set_object_class(self, obj_class: int):
        """
        Actualiza la clase del objeto de forma asíncrona. 
        Protegido por `class_locked` para evitar falsas clasificaciones inducidas por oclusión del brazo durante el Pick.
        """
        if not self.class_locked:
            self.current_obj_class = obj_class

    def start_dynamic_mode(self, initial_pos: list):
        """Inicializa la máquina de estados en modo continuo."""
        if self.estop_active: return False
        
        self.guard_xyz = list(initial_pos)
        
        if self.is_in_guard_position():
            self.activate_listening_mode()
        else:
            self.auto_step = 1
            self.tab_dynamic.lbl_dyn_status.setText(
                f"📍 Moviéndose a guardia (Meta: X={self.guard_xyz[0]:.1f}, Y={self.guard_xyz[1]:.1f}, Z={self.guard_xyz[2]:.1f} ±2mm)..."
            )
            self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #3498db; color: white; padding: 10px; font-weight: bold;")
            self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=1.2)

        return True

    def process_dynamic_vision(self, vel_mm_s: float, y_actual: float, flag_medido: float):
        """
        Procesa la trama de telemetría determinista desde el nodo de visión en tiempo real.
        Maneja el cálculo de intercepción dinámica y el cerrojo de clasificación espacial.
        """
        if self.estop_active:
            return

        # 1. Seguimiento Continuo de Posición
        self.last_seen_y = y_actual

        # 2. Cerrojo de Clasificación Espacial (Y ~= 0)
        # Bloqueamos la clase del objeto cuando pasa por el centro exacto de la cámara (Y=0)
        # para garantizar la mínima distorsión de perspectiva y cero oclusión mecánica por parte del manipulador.
        if -20.0 <= y_actual <= 20.0 and not self.class_locked:
            self.class_locked = True
            clase_str = "Cubo" if self.current_obj_class == 1 else "Cono"
            print(f"[VISION LOCK] Clase fijada como: {clase_str} en Y={y_actual:.1f} mm. Previniendo ruido por oclusión.")

        # 3. Lógica de Intercepción (Solo se evalúa si está en estado de Guardia)
        if self.auto_step != 2:
            return

        if flag_medido == 1.0 and vel_mm_s > 10.0:
            self.calculated_vel = vel_mm_s
            self.tab_dynamic.lbl_vel_val.setText(f"Velocidad: {self.calculated_vel:.1f} mm/s")

            # Cálculo de factibilidad cinemática con un margen de anticipación de 0.15s (compensación electromagnética)
            distancia_restante = self.Y_TARGET_PICK - y_actual
            t_calculado = (distancia_restante / self.calculated_vel) - 0.15
            t_objetivo = max(self.T_MIN_ROBOT, t_calculado)

            if t_objetivo >= self.T_MIN_ROBOT:
                self.auto_step = 4
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"🎯 INTERCEPTANDO | X={self.X_CONVEYOR:.1f} Y=+120 mm (T={t_objetivo:.2f}s, V={self.calculated_vel:.1f} mm/s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #8e44ad; color: white; padding: 10px; font-weight: bold;")
                self.ros_thread.send_ctraj_cmd(self.X_CONVEYOR, self.Y_TARGET_PICK, self.Z_PICK, duration=t_objetivo)
            else:
                # Aborto por inalcanzabilidad física (velocidad de cinta excesiva)
                self.auto_step = 8
                self.class_locked = False  # Liberar el cerrojo para evaluar el próximo objeto
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"🚫 Aborto Cinemático: Imposible interceptar (T_disp: {t_objetivo:.2f}s < {self.T_MIN_ROBOT:.2f}s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #c0392b; color: white; padding: 10px; font-weight: bold;")
                QTimer.singleShot(2500, self.activate_listening_mode)

    def execute_place_sequence(self):
        """Enruta el efector final hacia la caja de clasificación correspondiente basándose en la clase bloqueada."""
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

        self.tab_dynamic.lbl_dyn_status.setText(f"🚀 Traslado ({forma_txt}) hacia X={self.current_place_x}, Y={self.current_place_y}...")
        self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, 110.0, duration=1.6)

    def process_planner_status(self, code: int, main_window):
        """
        Callback impulsado por el estado del planificador de trayectorias embebido en STM32.
        Maneja la ejecución determinista y secuencial de las trayectorias multi-segmento.
        """
        if code == 2: # Estado 2 = Trayectoria Completada Exitosamente
            if self.auto_step == 4:
                # Fase 1: Elevación vertical pura para prevenir fuerzas de corte contra la cinta en movimiento
                self.auto_step = 45
                self.tab_dynamic.lbl_dyn_status.setText("⬆️ Elevando pieza verticalmente (Lift)...")
                self.ros_thread.send_ctraj_cmd(self.X_CONVEYOR, self.Y_TARGET_PICK, self.Z_LIFT, duration=0.6)

            elif self.auto_step == 45:
                # Fase 2: Verificación de Agarre y Enrutamiento
                # Si la cámara detecta que el objeto superó Y=135mm, el agarre magnético falló. Se activa el failsafe.
                if self.last_seen_y > 135.0:
                    self.auto_step = 7
                    main_window.magnet_active = False
                    self.ros_thread.send_magnet(False)
                    
                    self.tab_dynamic.lbl_dyn_status.setText("⚠️ FALLA DE AGARRE: Pieza detectada en cinta. Abortando traslado...")
                    self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #e74c3c; color: white; padding: 10px; font-weight: bold;")
                    
                    # Retorno evasivo a guardia (alta velocidad)
                    self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=0.9)
                else:
                    # Pick exitoso, procedemos a la caja de depósito
                    self.execute_place_sequence()
            
            elif self.auto_step == 5:
                # Fase 3: Descenso final en la caja de clasificación
                self.auto_step = 6
                self.tab_dynamic.lbl_dyn_status.setText(f"⬇️ Descenso final en Z={self.current_place_z}...")
                self.ros_thread.send_ctraj_cmd(self.current_place_x, self.current_place_y, self.current_place_z, duration=0.8)

            elif self.auto_step == 6:
                # Fase 4: Liberación del objeto y retorno a guardia
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                self.tab_dynamic.lbl_dyn_status.setText("🔓 Pieza depositada. Retornando a guardia...")
                self.tab_dynamic.lbl_dyn_status.setStyleSheet("background-color: #9b59b6; color: white; padding: 10px; font-weight: bold;")
                self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=1.4)

            elif self.auto_step == 7:
                # Fase 5: Reevaluar la matriz de tolerancia de guardia antes de habilitar el modo de escucha
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(self.guard_xyz[0], self.guard_xyz[1], self.guard_xyz[2], duration=0.6)