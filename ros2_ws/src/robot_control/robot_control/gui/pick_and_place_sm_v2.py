"""Máquina de estados finita determinista (FSM V2) para intercepción dinámica continua[cite: 40].

Coordina el ciclo dinámico autónomo evaluando factibilidad cinemática en tiempo real,
sincronizando el instante de arribo con el perfil quíntico generado en el microcontrolador
y gestionando el protocolo de estados del planificador (/planner/traj_status)[cite: 40].

Estados del Autómata:
    0: IDLE / Sistema detenido o en espera de inicio[cite: 40].
    1: Traslado articular hacia posición de guardia nominal[cite: 40].
    2: En guardia validada (±2.0 mm) | Imán ON | Espera de objeto en cinta[cite: 40].
    4: Intercepción síncrona en movimiento quíntico continuo[cite: 40].
    45: Elevación vertical pura (Vertical Lift) para mitigar fuerzas de corte[cite: 40].
    5: Desplazamiento elevado hacia tolva de descarga según clasificación[cite: 40].
    6: Descenso final y corte de excitación magnética[cite: 40].
    7: Retorno veloz a guardia tras depósito o falla de sujeción[cite: 40].
    8: Descarte de intercepción por inviabilidad dinámica (velocidad excesiva)[cite: 40].
"""

from typing import List
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread
from robot_control.gui.kinematics import inverse_kinematics


class TrajectoryStatus:
    """Códigos de estado sincronizados con el firmware STM32."""
    ABORT_ESTOP = -3
    ERROR_IK_PATH = -2
    ERROR_IK_START = -1
    IDLE = 0
    RUNNING = 1
    SUCCESS = 2
    TIMEOUT = 3


class PickAndPlaceSMV2:
    """Controlador de estados determinista para ciclo continuo autónomo.

    Estados del autómata:
        0: IDLE / Celda detenida o en reposo dinámico.
        1: Viaje de posicionamiento hacia guardia inicial (lazo cerrado).
        2: En Guardia validada (±2.0 mm) | Imán ON | Espera de nuevo objeto.
        4: Trayectoria quíntica hacia meta de intercepción activa.
        45: Elevación vertical pura (Vertical Lift) para mitigar fuerzas de corte.
        5: Traslado elevado hacia contenedor según clase clasificada.
        6: Descenso final y descarga del objeto.
        7: Retorno veloz a guardia tras depósito o falla de agarre.
        8: Aborto cinemático temporal por velocidad excesiva de la cinta (No factible).
    """

    # Parámetros espaciales unificados [mm]
    X_CONVEYOR: float = 215.0       # Centro longitudinal sobre la cinta transportadora [mm]
    Y_TARGET_PICK: float = 120.0    # Cota de intercepción dinámica síncrona [mm]
    Z_PICK: float = 76.0            # Altura de contacto magnético firme sobre la pieza [mm]
    Z_LIFT: float = 125.0           # Despegue vertical de seguridad [mm]
    
    # Parámetros dinámicos congruentes con firmware
    TRAJ_MIN_DURATION_S: float = 0.60  # Cota temporal inferior absoluta del firmware [s]
    SAFE_FREQ_HZ: float = 850.0        # 85% de MAX_V_HZ (1000 Hz)
    DEG_TO_STEPS: float = 1600.0 / 360.0
    GEAR_RATIOS: List[float] = [108.0 / 19.0, 32.0 / 12.0, 32.0 / 12.0]
    
    T_ANTICIPO: float = 0.15        # Adelanto temporal por retardo electromagnético [s]
    POS_TOLERANCE: float = 2.0      # Esfera de tolerancia de guardia en lazo cerrado [mm]
    Y_DROP_FAILSAFE: float = 135.0  # Cota de verificación óptica de pérdida de pieza [mm]

    def __init__(self, ros_thread: ROS2Thread, tab_dynamic) -> None:
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic

        self.auto_step: int = 0
        self.estop_active: bool = False

        self.guard_xyz: List[float] = [self.X_CONVEYOR, -75.0, 100.0]
        self.current_robot_xyz: List[float] = list(self.guard_xyz)
        self.current_robot_q: List[float] = [0.0, 90.0, 0.0]

        self.calculated_vel: float = 0.0
        self.last_seen_y: float = 0.0

        # Clasificación morfológica con cerrojo espacial
        self.current_obj_class: int = 1  # 1: Cubo, 2: Cono
        self.class_locked: bool = False
        self.current_place_x: float = 115.0
        self.current_place_y: float = 115.0
        self.current_place_z: float = 42.0

    def is_active(self) -> bool:
        """Indica si el ciclo dinámico autónomo está en ejecución."""
        return self.auto_step > 0

    def set_estop(self, state: bool) -> None:
        self.estop_active = state
        if state:
            self.auto_step = 0
            # Ocultar cartel de clasificación si se activa la emergencia
            self.tab_dynamic.group_class_status.setVisible(False)

    def set_robot_telemetry(self, q_deg: List[float], pos_xyz: List[float]) -> None:
        """Actualiza posición articular y cartesiana medida por los encoders."""
        self.current_robot_q = list(q_deg)
        self.current_robot_xyz = list(pos_xyz)
        if self.auto_step in [1, 7]:
            if self.is_in_guard_position():
                self.activate_listening_mode()

    def is_in_guard_position(self) -> bool:
        dx = abs(self.current_robot_xyz[0] - self.guard_xyz[0])
        dy = abs(self.current_robot_xyz[1] - self.guard_xyz[1])
        dz = abs(self.current_robot_xyz[2] - self.guard_xyz[2])
        return (
            (dx <= self.POS_TOLERANCE)
            and (dy <= self.POS_TOLERANCE)
            and (dz <= self.POS_TOLERANCE)
        )

    def activate_listening_mode(self) -> None:
        self.auto_step = 2
        self.class_locked = False
        self.ros_thread.send_magnet(True)
        
        # Mostrar y actualizar el cartel de clasificación al entrar en modo automático
        self.tab_dynamic.group_class_status.setVisible(True)
        self.tab_dynamic.lbl_class_status.setText("Clasificación: Esperando pieza...")
        self.tab_dynamic.lbl_class_status.setStyleSheet(
            "background-color: #f1c40f; color: #2c3e50; padding: 10px; font-weight: bold;"
        )

        self.tab_dynamic.lbl_dyn_status.setText(
            "Guardia OK (±2.0 mm) | Imán ON | Esperando ingreso de objeto..."
        )
        self.tab_dynamic.lbl_dyn_status.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 10px; font-weight: bold;"
        )

    def set_object_class(self, obj_class: int) -> None:
        if not self.class_locked:
            self.current_obj_class = obj_class
            
            # Actualizar el cartel de clasificación en tiempo real si el modo automático está activo
            if self.is_active():
                nombre_pieza = "Cubo" if obj_class == 1 else "Cono"
                color_bg = "#2980b9" if obj_class == 1 else "#d35400"
                self.tab_dynamic.lbl_class_status.setText(f"Clasificación Activa: {nombre_pieza}")
                self.tab_dynamic.lbl_class_status.setStyleSheet(
                    f"background-color: {color_bg}; color: white; padding: 10px; font-weight: bold;"
                )

    def calculate_t_min_robot(self, target_xyz: List[float]) -> float:
        """Calcula el tiempo mínimo viable idéntico a la fórmula analítica del firmware."""
        q_target, reachable = inverse_kinematics(target_xyz[0], target_xyz[1], target_xyz[2])
        if not reachable:
            return 999.0

        t_min = self.TRAJ_MIN_DURATION_S
        for i in range(3):
            dq = abs(q_target[i] - self.current_robot_q[i])
            if dq > 0.01:
                t_req = (1.875 * dq * self.DEG_TO_STEPS * self.GEAR_RATIOS[i]) / self.SAFE_FREQ_HZ
                if t_req > t_min:
                    t_min = t_req
        return t_min

    def start_dynamic_mode(self, initial_pos: List[float]) -> bool:
        if self.estop_active:
            return False

        self.guard_xyz = list(initial_pos)

        if self.is_in_guard_position():
            self.activate_listening_mode()
        else:
            self.auto_step = 1
            # Mostrar el cartel indicando el movimiento hacia guardia
            self.tab_dynamic.group_class_status.setVisible(True)
            self.tab_dynamic.lbl_class_status.setText("Clasificación: Posicionándose en Guardia...")
            self.tab_dynamic.lbl_class_status.setStyleSheet(
                "background-color: #3498db; color: white; padding: 10px; font-weight: bold;"
            )

            self.tab_dynamic.lbl_dyn_status.setText(
                f"Moviéndose a guardia (Meta: X={self.guard_xyz[0]:.1f}, "
                f"Y={self.guard_xyz[1]:.1f}, Z={self.guard_xyz[2]:.1f} ±2mm)..."
            )
            self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                "background-color: #3498db; color: white; padding: 10px; font-weight: bold;"
            )
            self.ros_thread.send_ctraj_cmd(
                self.guard_xyz[0],
                self.guard_xyz[1],
                self.guard_xyz[2],
                duration=1.2,
            )
        return True

    def process_dynamic_vision(
        self, vel_mm_s: float, y_actual: float, flag_medido: float
    ) -> None:
        if self.estop_active or not self.is_active():
            return

        self.last_seen_y = y_actual

        # Cerrojo de clasificación espacial en el centro de la lente [-40 mm, -20 mm]
        if -40.0 <= y_actual <= -20.0 and not self.class_locked:
            self.class_locked = True

        if self.auto_step != 2:
            return

        if flag_medido == 1.0 and vel_mm_s > 10.0:
            self.calculated_vel = vel_mm_s
            self.tab_dynamic.lbl_vel_val.setText(
                f"Velocidad: {self.calculated_vel:.1f} mm/s"
            )

            # Tiempo disponible del objeto hasta el punto de agarre
            distancia_restante = self.Y_TARGET_PICK - y_actual
            t_disponible = (distancia_restante / self.calculated_vel) - self.T_ANTICIPO

            # Tiempo analítico mínimo real demandado por el robot
            t_min_robot = self.calculate_t_min_robot([self.X_CONVEYOR, self.Y_TARGET_PICK, self.Z_PICK])

            if t_disponible >= t_min_robot:
                self.auto_step = 4
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"INTERCEPTANDO | X={self.X_CONVEYOR:.1f} Y=+{self.Y_TARGET_PICK:.1f} mm "
                    f"(T={t_disponible:.2f}s, T_min={t_min_robot:.2f}s, V={self.calculated_vel:.1f} mm/s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #8e44ad; color: white; padding: 10px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    self.X_CONVEYOR,
                    self.Y_TARGET_PICK,
                    self.Z_PICK,
                    duration=t_disponible,
                )
            else:
                # Aborto por no factibilidad cinemática
                self.auto_step = 8
                self.class_locked = False
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"NO FACTIBLE: Velocidad excesiva (T_disp: {t_disponible:.2f}s < T_min: {t_min_robot:.2f}s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #c0392b; color: white; padding: 10px; font-weight: bold;"
                )
                QTimer.singleShot(2500, self.activate_listening_mode)

    def execute_place_sequence(self) -> None:
        if self.estop_active:
            return
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
            f"Traslado ({forma_txt}) hacia X={self.current_place_x}, Y={self.current_place_y}..."
        )
        self.ros_thread.send_ctraj_cmd(
            self.current_place_x, self.current_place_y, 110.0, duration=1.6
        )

    def process_planner_status(self, code: int, main_window) -> None:
        """Máquina de estados dirigida por eventos del firmware STM32.
        
        Solo actúa si el autómata dinámico se encuentra en ejecución (auto_step > 0).
        """
        if not self.is_active():
            return

        # 1. Manejo de Errores Críticos y Abortos Embebidos en Modo Dinámico
        if code in [
            TrajectoryStatus.ERROR_IK_START,
            TrajectoryStatus.ERROR_IK_PATH,
            TrajectoryStatus.ABORT_ESTOP,
        ]:
            self.tab_dynamic.lbl_dyn_status.setText(
                f"ERROR EN FIRMWARE (Código {code}): Abortando ciclo dinámico..."
            )
            self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                "background-color: #c0392b; color: white; padding: 10px; font-weight: bold;"
            )
            self.auto_step = 0
            self.tab_dynamic.group_class_status.setVisible(False)
            main_window.magnet_active = False
            self.ros_thread.send_magnet(False)
            return

        # 2. Trayectoria en curso
        if code == TrajectoryStatus.RUNNING:
            return

        # 3. Manejo de TIMEOUT de asentamiento (Código 3): desvío > 0.45° tras 0.8s
        if code == TrajectoryStatus.TIMEOUT:
            print(f"[WARN FSM] Trayectoria finalizada por TIMEOUT en auto_step={self.auto_step}")
            if self.auto_step == 4:
                # Falla crítica en la captura: abortar descenso y volver a guardia
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                self.tab_dynamic.lbl_dyn_status.setText(
                    "TIMEOUT EN PICK: No convergió a tiempo. Retornando a guardia..."
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #d35400; color: white; padding: 10px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    self.guard_xyz[0],
                    self.guard_xyz[1],
                    self.guard_xyz[2],
                    duration=1.0,
                )
                return

        # 4. Trayectoria Completada (SUCCESS = 2 o continuación controlada tras TIMEOUT en traslados)
        if code in [TrajectoryStatus.SUCCESS, TrajectoryStatus.TIMEOUT]:
            if self.auto_step == 4:
                # Fase 1: Elevación vertical pura (Z_lift = 125 mm)
                self.auto_step = 45
                self.tab_dynamic.lbl_dyn_status.setText("Elevando pieza verticalmente (Lift)...")
                self.ros_thread.send_ctraj_cmd(
                    self.X_CONVEYOR,
                    self.Y_TARGET_PICK,
                    self.Z_LIFT,
                    duration=0.6,
                )

            elif self.auto_step == 45:
                # Fase 2: Verificación óptica de pérdida de pieza
                if self.last_seen_y > self.Y_DROP_FAILSAFE:
                    self.auto_step = 7
                    main_window.magnet_active = False
                    self.ros_thread.send_magnet(False)
                    self.tab_dynamic.lbl_dyn_status.setText(
                        "FALLA DE AGARRE: Pieza en cinta. Retorno rápido a guardia..."
                    )
                    self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                        "background-color: #e74c3c; color: white; padding: 10px; font-weight: bold;"
                    )
                    self.ros_thread.send_ctraj_cmd(
                        self.guard_xyz[0],
                        self.guard_xyz[1],
                        self.guard_xyz[2],
                        duration=0.9,
                    )
                else:
                    self.execute_place_sequence()

            elif self.auto_step == 5:
                # Fase 3: Descenso final en el contenedor
                self.auto_step = 6
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"Descenso final en Z={self.current_place_z} mm..."
                )
                self.ros_thread.send_ctraj_cmd(
                    self.current_place_x,
                    self.current_place_y,
                    self.current_place_z,
                    duration=0.8,
                )

            elif self.auto_step == 6:
                # Fase 4: Desenergización y retorno a guardia
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                self.tab_dynamic.lbl_dyn_status.setText(
                    "Pieza depositada. Retornando a guardia..."
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #9b59b6; color: white; padding: 10px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    self.guard_xyz[0],
                    self.guard_xyz[1],
                    self.guard_xyz[2],
                    duration=1.4,
                )

            elif self.auto_step == 7:
                # Fase 5: Reingreso a guardia
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(
                        self.guard_xyz[0],
                        self.guard_xyz[1],
                        self.guard_xyz[2],
                        duration=0.6,
                    )