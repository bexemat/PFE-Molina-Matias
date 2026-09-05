"""Máquina de estados finitos para intercepción dinámica síncrona en cinta transportadora.

Coordina la sincronización temporal MRU, la anticipación electromagnética,
el control de posición de guardia en lazo cerrado (±2.0 mm), el despegue
vertical puro (Z_lift) y el retorno rápido ante fallo de agarre.
"""

from typing import List, Optional
from PyQt5.QtCore import QTimer
from robot_control.gui.ros_thread import ROS2Thread


class PickAndPlaceSM:
    """Máquina de estados determinista para el ciclo dinámico de pick-and-place.

    Estados del ciclo:
        0: Inactivo / Parada de Emergencia.
        1: Desplazamiento hacia posición de guardia inicial.
        2: En Guardia validada (±2.0 mm) | Electroimán ON | Espera de objeto.
        3: Intercepción sincronizada quíntica en cinta (Y_pick).
        45: Elevación vertical pura (Vertical Lift) y validación de agarre.
        5: Traslado elevado hacia contenedor clasificado (Cubo/Cono).
        6: Descenso final en contenedor de descarga.
        7: Retorno seguro a posición de guardia.
        8: Aborto cinemático temporal por velocidad excesiva.
    """

    # Parámetros geométricos y cotas cinemáticas nominales [mm]
    X_CONVEYOR: float = 215.0       # Coordenada transversal sobre la cinta [mm]
    Y_TARGET_PICK: float = 120.0    # Coordenada longitudinal de intercepción [mm]
    Z_PICK: float = 78.0            # Altura de contacto sobre la superficie del objeto [mm]
    Z_LIFT: float = 125.0           # Cota de despegue vertical para aislar fuerzas de corte [mm]
    T_MIN_ROBOT: float = 0.85       # Límite inferior dinámico de aceleración del robot [s]
    T_ANTICIPO: float = 0.15        # Compensación de magnetización plena del solenoide [s]
    POS_TOLERANCE: float = 2.0      # Esfera de tolerancia en guardia en lazo cerrado [mm]
    Y_DROP_FAILSAFE: float = 135.0  # Cota límite de detección de desprendimiento de pieza [mm]

    def __init__(self, ros_thread: ROS2Thread, tab_dynamic) -> None:
        """Inicializa la máquina de estados y registra referencias de control.

        Args:
            ros_thread (ROS2Thread): Hilo intermediario de comunicaciones ROS 2.
            tab_dynamic (DynamicTab): Pestaña visual asociada para notificaciones de estado.
        """
        self.ros_thread = ros_thread
        self.tab_dynamic = tab_dynamic

        self.auto_step: int = 0
        self.estop_active: bool = False

        self.guard_xyz: List[float] = [self.X_CONVEYOR, -75.0, 100.0]
        self.current_robot_xyz: List[float] = list(self.guard_xyz)

        self.calculated_vel: float = 0.0
        self.last_seen_y: float = 0.0

        # Gestión de clasificación morfológica y cerrojo (Latch)
        self.current_obj_class: int = 1  # 1: Cubo, 2: Cono
        self.class_locked: bool = False
        self.current_place_x: float = 115.0
        self.current_place_y: float = 115.0
        self.current_place_z: float = 42.0

    def set_estop(self, state: bool) -> None:
        """Conmuta el estado de parada de emergencia en la máquina de estados."""
        self.estop_active = state
        if state:
            self.auto_step = 0

    def set_robot_pos(self, pos_xyz: List[float]) -> None:
        """Actualiza la posición cartesiana y verifica la condición de guardia.

        Args:
            pos_xyz (List[float]): Posición actual del robot [X, Y, Z] en milímetros [mm].
        """
        self.current_robot_xyz = list(pos_xyz)
        if self.auto_step in [1, 7]:
            if self.is_in_guard_position():
                self.activate_listening_mode()

    def is_in_guard_position(self) -> bool:
        """Valida si el efector se encuentra dentro de la esfera de tolerancia en guardia.

        Returns:
            bool: True si el error euclídeo en los 3 ejes es <= POS_TOLERANCE (±2.0 mm).
        """
        dx = abs(self.current_robot_xyz[0] - self.guard_xyz[0])
        dy = abs(self.current_robot_xyz[1] - self.guard_xyz[1])
        dz = abs(self.current_robot_xyz[2] - self.guard_xyz[2])
        return (
            (dx <= self.POS_TOLERANCE)
            and (dy <= self.POS_TOLERANCE)
            and (dz <= self.POS_TOLERANCE)
        )

    def activate_listening_mode(self) -> None:
        """Transiciona al estado de espera activa con imán energizado en guardia."""
        self.auto_step = 2
        self.class_locked = False
        self.ros_thread.send_magnet(True)
        self.tab_dynamic.lbl_dyn_status.setText(
            "Guardia OK (±2.0 mm) | Imán ON | Esperando objeto..."
        )
        self.tab_dynamic.lbl_dyn_status.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 10px; font-weight: bold;"
        )

    def set_calculated_vel(self, vel: float) -> None:
        """Asigna la velocidad lineal estimada sobre la cinta transportadora [mm/s]."""
        self.calculated_vel = vel

    def set_object_class(self, obj_class: int) -> None:
        """Actualiza la clase morfológica si el cerrojo óptico no está trabado."""
        if not self.class_locked:
            self.current_obj_class = obj_class

    def execute_interception(
        self, current_pos: Optional[List[float]] = None
    ) -> bool:
        """Inicia el ciclo dinámico enviando el efector a la posición de guardia inicial.

        Returns:
            bool: True si la orden fue admitida; False si el E-Stop está activo.
        """
        if self.estop_active:
            return False

        if self.is_in_guard_position():
            self.activate_listening_mode()
        else:
            self.auto_step = 1
            self.tab_dynamic.lbl_dyn_status.setText("Posicionando en guardia...")
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

    def trigger_ctraj_interception(self, y_actual: float) -> None:
        """Evalúa la posición instantánea, traba la clase y dispara la trayectoria quíntica.

        Args:
            y_actual (float): Coordenada longitudinal Y actual del objeto [mm].
        """
        if self.estop_active:
            return
        self.last_seen_y = y_actual

        # Cerrojo de clasificación en el centro óptico [-20 mm, +20 mm]
        if -20.0 <= y_actual <= 20.0 and not self.class_locked:
            self.class_locked = True

        if self.auto_step != 2:
            return

        if self.calculated_vel <= 0:
            return

        distancia_restante = self.Y_TARGET_PICK - y_actual
        if distancia_restante <= 0:
            return

        # Sincronización temporal MRU compensando respuesta electromagnética
        t_calculado = (
            distancia_restante / self.calculated_vel
        ) - self.T_ANTICIPO
        t_objetivo = max(self.T_MIN_ROBOT, t_calculado)

        if t_calculado >= self.T_MIN_ROBOT:
            self.auto_step = 3
            self.tab_dynamic.lbl_dyn_status.setText(
                f"INTERCEPTANDO | Y=+{self.Y_TARGET_PICK:.1f} mm "
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
            # Descarte cinemático por velocidad de cinta inviable
            self.auto_step = 8
            self.class_locked = False
            self.tab_dynamic.lbl_dyn_status.setText(
                "Velocidad inviable: Tiempo insuficiente para intercepción."
            )
            self.tab_dynamic.lbl_dyn_status.setStyleSheet(
                "background-color: #c0392b; color: white; padding: 10px; font-weight: bold;"
            )
            QTimer.singleShot(2500, self.activate_listening_mode)

    def execute_place_sequence(self) -> None:
        """Planifica el traslado hacia el contenedor de clasificación."""
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
        self.tab_dynamic.lbl_dyn_status.setStyleSheet(
            "background-color: #2980b9; color: white; padding: 10px; font-weight: bold;"
        )
        # Traslado a cota elevada de seguridad (Z=110 mm)
        self.ros_thread.send_ctraj_cmd(
            self.current_place_x, self.current_place_y, 110.0, duration=1.6
        )

    def process_planner_status(self, code: int, main_window) -> None:
        """Gestiona las transiciones del ciclo impulsadas por eventos de la STM32.

        Args:
            code (int): Código de retorno del planificador embebido (1, 2 o -1).
            main_window: Referencia a la ventana gráfica principal.
        """
        if code == 1:
            main_window.lbl_planner_state.setText(
                "Estado: TRAYECTORIA EN EJECUCIÓN..."
            )
        elif code == 2:
            main_window.lbl_planner_state.setText("Estado: COMPLETADO")

            if self.auto_step == 1:
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(
                        self.guard_xyz[0],
                        self.guard_xyz[1],
                        self.guard_xyz[2],
                        duration=0.6,
                    )

            elif self.auto_step == 3:
                # Paso 1: Elevación vertical pura (Vertical Lift) para despegar la pieza
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
                # Paso 2: Verificación óptica de retención magnética
                if self.last_seen_y > self.Y_DROP_FAILSAFE:
                    # Desprendimiento detectado: aborto y retorno rápido
                    self.auto_step = 7
                    main_window.magnet_active = False
                    self.ros_thread.send_magnet(False)
                    main_window.btn_magnet.setText("ELECTROIMÁN: APAGADO")
                    self.tab_dynamic.lbl_dyn_status.setText(
                        "FALLA DE AGARRE: Objeto en cinta. Retorno rápido a guardia..."
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
                # Paso 3: Descenso final en la caja asignada
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
                # Paso 4: Desenergización y retorno a guardia
                self.auto_step = 7
                main_window.magnet_active = False
                self.ros_thread.send_magnet(False)
                main_window.btn_magnet.setText("ELECTROIMÁN: APAGADO")
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
                # Paso 5: Confirmación de guardia
                if self.is_in_guard_position():
                    self.activate_listening_mode()
                else:
                    self.ros_thread.send_ctraj_cmd(
                        self.guard_xyz[0],
                        self.guard_xyz[1],
                        self.guard_xyz[2],
                        duration=0.6,
                    )

            QTimer.singleShot(
                50, lambda: setattr(main_window, "is_tracking_yz", False)
            )