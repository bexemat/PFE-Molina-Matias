"""Máquina de estados V2 para intercepción dinámica en 1 paso y flujo continuo.

Elimina la necesidad de calibración manual previa. Integra la estimación
de velocidad en tiempo real de cada pieza, sincroniza la llegada con el
planificador quíntico en la STM32, ejecuta despegue vertical para aislar
esfuerzos cortantes y supervisa el agarre de forma ininterrumpida.
"""

from typing import List
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread


class PickAndPlaceSMV2:
    """Controlador de estados determinista para ciclo continuo autónomo.

    Estados del autómata:
        0: IDLE / Celda detenida o en parada de emergencia.
        1: Viaje de posicionamiento hacia guardia inicial (lazo cerrado).
        2: En Guardia validada (±2.0 mm) | Imán ON | Espera de nuevo objeto.
        4: Trayectoria quíntica hacia meta de intercepción activa.
        45: Elevación vertical pura (Vertical Lift) para mitigar fuerzas de corte.
        5: Traslado elevado hacia contenedor según clase clasificada.
        6: Descenso final y descarga del objeto.
        7: Retorno veloz a guardia tras depósito o falla de agarre.
        8: Aborto cinemático temporal por velocidad excesiva de la cinta.
    """

    # Parámetros mecánicos y de intercepción [mm]
    X_CONVEYOR: float = 215.0       # Centro longitudinal sobre la cinta transportadora [mm]
    Y_TARGET_PICK: float = 120.0    # Cota de intercepción dinámica síncrona [mm]
    Z_PICK: float = 78.0            # Altura de contacto magnético firme sobre la pieza [mm]
    Z_LIFT: float = 125.0           # Despegue vertical de seguridad [mm]
    T_MIN_ROBOT: float = 0.85       # Límite dinámico temporal mínimo viable del robot [s]
    T_ANTICIPO: float = 0.15        # Adelanto temporal por retardo electromagnético [s]
    POS_TOLERANCE: float = 2.0      # Esfera de tolerancia de guardia en lazo cerrado [mm]
    Y_DROP_FAILSAFE: float = 135.0  # Cota de verificación óptica de pérdida de pieza [mm]

    def __init__(self, ros_thread: ROS2Thread, tab_dynamic) -> None:
        """Inicializa la máquina de estados desacoplada de la interfaz gráfica.

        Args:
            ros_thread (ROS2Thread): Hilo puente de eventos ROS 2.
            tab_dynamic: Pestaña visual asociada (DynamicContinuousTab).
        """
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic

        self.auto_step: int = 0
        self.estop_active: bool = False

        self.guard_xyz: List[float] = [self.X_CONVEYOR, -75.0, 100.0]
        self.current_robot_xyz: List[float] = list(self.guard_xyz)

        self.calculated_vel: float = 0.0
        self.last_seen_y: float = 0.0

        # Clasificación morfológica con cerrojo espacial
        self.current_obj_class: int = 1  # 1: Cubo, 2: Cono
        self.class_locked: bool = False
        self.current_place_x: float = 115.0
        self.current_place_y: float = 115.0
        self.current_place_z: float = 42.0

    def set_estop(self, state: bool) -> None:
        """Sobrescribe el estado ante parada de emergencia global."""
        self.estop_active = state
        if state:
            self.auto_step = 0

    def set_robot_pos(self, pos_xyz: List[float]) -> None:
        """Actualiza la telemetría articular proyectada y verifica condición de reposo."""
        self.current_robot_xyz = list(pos_xyz)
        if self.auto_step in [1, 7]:
            if self.is_in_guard_position():
                self.activate_listening_mode()

    def is_in_guard_position(self) -> bool:
        """Comprueba si el efector se encuentra dentro de la tolerancia de ±2.0 mm."""
        dx = abs(self.current_robot_xyz[0] - self.guard_xyz[0])
        dy = abs(self.current_robot_xyz[1] - self.guard_xyz[1])
        dz = abs(self.current_robot_xyz[2] - self.guard_xyz[2])
        return (
            (dx <= self.POS_TOLERANCE)
            and (dy <= self.POS_TOLERANCE)
            and (dz <= self.POS_TOLERANCE)
        )

    def activate_listening_mode(self) -> None:
        """Habilita la captura dinámica con electroimán pre-energizado en guardia."""
        self.auto_step = 2
        self.class_locked = False
        self.ros_thread.send_magnet(True)
        self.tab_dynamic.lbl_dyn_status.setText(
            "Guardia OK (±2.0 mm) | Imán ON | Esperando ingreso de objeto..."
        )
        self.tab_dynamic.lbl_dyn_status.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 10px; font-weight: bold;"
        )

    def set_object_class(self, obj_class: int) -> None:
        """Actualiza la morfología identificada si el cerrojo óptico está abierto."""
        if not self.class_locked:
            self.current_obj_class = obj_class

    def start_dynamic_mode(self, initial_pos: List[float]) -> bool:
        """Conduce el efector a la posición de guardia y arranca el ciclo continuo.

        Args:
            initial_pos (List[float]): Coordenadas de guardia [X, Y, Z] deseadas [mm].

        Returns:
            bool: True si la orden fue admitida; False si hay parada de emergencia.
        """
        if self.estop_active:
            return False

        self.guard_xyz = list(initial_pos)

        if self.is_in_guard_position():
            self.activate_listening_mode()
        else:
            self.auto_step = 1
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
        """Procesa paquetes deterministas de visión y resuelve sincronización temporal.

        Args:
            vel_mm_s (float): Velocidad lineal calculada en compuertas [mm/s].
            y_actual (float): Coordenada longitudinal Y instantánea [mm].
            flag_medido (float): 1.0 si la velocidad fue validada; 0.0 en caso contrario.
        """
        if self.estop_active:
            return

        self.last_seen_y = y_actual

        # Cerrojo de clasificación espacial en el centro de la lente [-20 mm, +20 mm]
        if -20.0 <= y_actual <= 20.0 and not self.class_locked:
            self.class_locked = True

        if self.auto_step != 2:
            return

        if flag_medido == 1.0 and vel_mm_s > 10.0:
            self.calculated_vel = vel_mm_s
            self.tab_dynamic.lbl_vel_val.setText(
                f"Velocidad: {self.calculated_vel:.1f} mm/s"
            )

            # Modelo cinemático MRU con factor de anticipación
            distancia_restante = self.Y_TARGET_PICK - y_actual
            t_calculado = (
                distancia_restante / self.calculated_vel
            ) - self.T_ANTICIPO
            t_objetivo = max(self.T_MIN_ROBOT, t_calculado)

            if t_calculado >= self.T_MIN_ROBOT:
                self.auto_step = 4
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"INTERCEPTANDO | X={self.X_CONVEYOR:.1f} Y=+{self.Y_TARGET_PICK:.1f} mm "
                    f"(T={t_objetivo:.2f}s, V={self.calculated_vel:.1f} mm/s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #8e44ad; color: white; padding: 10px; font-weight: bold;"
                )
                self.ros_thread.send_ctraj_cmd(
                    self.X_CONVEYOR,
                    self.Y_TARGET_PICK,
                    self.Z_PICK,
                    duration=t_objetivo,
                )
            else:
                # Aborto por no factibilidad cinemática
                self.auto_step = 8
                self.class_locked = False
                self.tab_dynamic.lbl_dyn_status.setText(
                    f"Aborto Cinemático: Imposible interceptar (T_disp: {t_objetivo:.2f}s < {self.T_MIN_ROBOT:.2f}s)"
                )
                self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                    "background-color: #c0392b; color: white; padding: 10px; font-weight: bold;"
                )
                QTimer.singleShot(2500, self.activate_listening_mode)

    def execute_place_sequence(self) -> None:
        """Planifica la descarga clasificada según la morfología bloqueada."""
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
        """Máquina de estados dirigida por eventos de finalización de la STM32."""
        if code == 2:
            if self.auto_step == 4:
                # Fase 1: Elevación vertical pura (Z_lift = 125 mm)
                self.auto_step = 45
                self.tab_dynamic.lbl_dyn_status.setText(
                    "Elevando pieza verticalmente (Lift)..."
                )
                self.ros_thread.send_ctraj_cmd(
                    self.X_CONVEYOR,
                    self.Y_TARGET_PICK,
                    self.Z_LIFT,
                    duration=0.6,
                )

            elif self.auto_step == 45:
                # Fase 2: Verificación de agarre por cota longitudinal
                if self.last_seen_y > self.Y_DROP_FAILSAFE:
                    # Desprendimiento: aborto y retorno rápido
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
                # Fase 3: Descenso final en la caja
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
                # Fase 5: Validación de tolerancia estricta
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(
                        self.guard_xyz[0],
                        self.guard_xyz[1],
                        self.guard_xyz[2],
                        duration=0.6,
                    )