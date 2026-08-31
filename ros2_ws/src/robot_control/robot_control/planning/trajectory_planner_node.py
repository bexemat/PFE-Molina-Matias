#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Point
from extra_interfaces.msg import Trama

# Cinemática de tu GUI
from robot_control.gui.kinematics import forward_kinematics, inverse_kinematics


class TrajectoryPlannerNode(Node):
    """
    Nodo Planificador de Trayectorias de Alta Velocidad y Fluidez (LSPB Optimizado).
    """

    def __init__(self):
        super().__init__('trajectory_planner_node')

        # --- PARÁMETROS CONFIGURABLES ---
        self.declare_parameter('control_rate_hz', 100.0)  # 100 Hz (dt = 10 ms)
        self.rate_hz = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / self.rate_hz

        # --- LÍMITES FÍSICOS ELEVADOS ---
        STEPS_PER_REV = 200 * 8
        DEG_TO_STEPS = STEPS_PER_REV / 360.0
        self.gear_ratios = np.array([108.0 / 19.0, 32.0 / 12.0, 32.0 / 12.0]) 
        
        MAX_V_HZ = 800.0    # 800 Hz
        MAX_A_HZ_S = 1800.0 # 1800 Hz/s

        self.max_joint_vel = MAX_V_HZ / (DEG_TO_STEPS * self.gear_ratios)   # [deg/s]
        self.max_joint_acc = MAX_A_HZ_S / (DEG_TO_STEPS * self.gear_ratios)  # [deg/s^2]

        # --- ESTADO INTERNO ---
        self.current_q = [0.0, 90.0, 0.0]
        self.is_estop_active = False
        self.is_executing = False

        self.trajectory_q = []   
        self.trajectory_qd = []  
        self.total_duration = 0.0
        self.current_step = 0

        # --- PUBLICADORES Y SUSCRIPTORES ---
        self.cmd_pub = self.create_publisher(Trama, '/microROS/cmd', 10)
        self.status_pub = self.create_publisher(Int8, '/planner/traj_status', 10)

        self.feedback_sub = self.create_subscription(
            Point, '/microROS/angles', self._feedback_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, '/planner/emergency_stop', self._estop_callback, 10
        )
        self.jtraj_sub = self.create_subscription(
            Point, '/planner/jtraj_goal', self._jtraj_callback, 10
        )
        self.ctraj_sub = self.create_subscription(
            Point, '/planner/ctraj_goal', self._ctraj_callback, 10
        )
        # NUEVO TÓPICO DEDICADO PARA LA PESTAÑA DINÁMICA SÍNCRONA
        self.sync_ctraj_sub = self.create_subscription(
            Point, '/planner/sync_ctraj_goal', self._sync_ctraj_callback, 10
        )

        # Bucle periódico de control (10 ms)
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(f"⚡ Planificador Rápido y Fluido (LSPB) iniciado a {self.rate_hz} Hz.")

    def _calculate_dynamic_duration(self, q_start: list, q_stop: list) -> float:
        delta_q = np.abs(np.array(q_stop) - np.array(q_start))
        t_vel_req = (1.176 * delta_q) / self.max_joint_vel
        t_acc_req = np.sqrt((7.843 * delta_q) / self.max_joint_acc)
        t_min_needed = max(np.max(t_vel_req), np.max(t_acc_req))
        return float(max(0.05, t_min_needed * 1.02))

    @staticmethod
    def _trapezoidal_scaling(tau: float) -> tuple:
        tau = np.clip(tau, 0.0, 1.0)
        alpha = 0.15  
        a_norm = 1.0 / (alpha * (1.0 - alpha))  
        v_cruise = 1.0 / (1.0 - alpha)           

        if tau <= alpha:
            s = 0.5 * a_norm * (tau**2)
            ds_dtau = a_norm * tau
        elif tau <= (1.0 - alpha):
            s = 0.5 * a_norm * (alpha**2) + v_cruise * (tau - alpha)
            ds_dtau = v_cruise
        else:
            s = 1.0 - 0.5 * a_norm * ((1.0 - tau)**2)
            ds_dtau = a_norm * (1.0 - tau)

        return s, ds_dtau

    def _generate_jtraj(self, q_start: list, q_stop: list, duration: float):
        num_steps = int(max(1, duration * self.rate_hz))
        q_start = np.array(q_start, dtype=np.float64)
        q_stop = np.array(q_stop, dtype=np.float64)
        delta_q = q_stop - q_start

        traj_q, traj_qd = [], []

        for i in range(num_steps):
            tau = i / max(1, num_steps - 1)
            s, ds_dtau = self._trapezoidal_scaling(tau)
            q_t = q_start + delta_q * s
            qd_t = (delta_q / duration) * ds_dtau
            traj_q.append(q_t.tolist())
            traj_qd.append(qd_t.tolist())

        return np.array(traj_q), np.array(traj_qd)

    def _generate_ctraj(self, q_start: list, target_xyz: list, duration: float):
        num_steps = int(max(1, duration * self.rate_hz))
        
        x0_mm, y0_mm, z0_mm, _ = forward_kinematics(q_start)
        p0 = np.array([x0_mm, y0_mm, z0_mm], dtype=np.float64)
        pf = np.array(target_xyz, dtype=np.float64)

        traj_q = []
        for i in range(num_steps):
            tau = i / max(1, num_steps - 1)
            s, _ = self._trapezoidal_scaling(tau)
            p_t = p0 + (pf - p0) * s

            q_t, reachable = inverse_kinematics(p_t[0], p_t[1], p_t[2], raw_float=True)
            if not reachable:
                self.get_logger().error(f"⚠️ Punto en t={i*self.dt:.2f}s inalcanzable: XYZ={p_t} mm")
                return None, None, False

            traj_q.append(q_t)

        traj_q = np.array(traj_q)
        
        traj_qd = np.zeros_like(traj_q)
        for i in range(1, num_steps - 1):
            traj_qd[i] = (traj_q[i + 1] - traj_q[i - 1]) / (2.0 * self.dt)

        traj_qd[0] = [0.0, 0.0, 0.0]
        traj_qd[-1] = [0.0, 0.0, 0.0]

        if num_steps > 5:
            for col in range(3):
                traj_qd[2:-2, col] = np.convolve(traj_qd[:, col], np.ones(5) / 5.0, mode='valid')

        return traj_q, traj_qd, True

    def _feedback_callback(self, msg: Point):
        self.current_q = [msg.x, msg.y, msg.z]

    def _estop_callback(self, msg: Bool):
        self.is_estop_active = msg.data
        if self.is_estop_active and self.is_executing:
            self._abort_trajectory()

    def _jtraj_callback(self, msg: Point):
        if self.is_estop_active: return
        q_start = list(self.current_q)
        q_stop = [msg.x, msg.y, msg.z]

        duration = self._calculate_dynamic_duration(q_start, q_stop)
        self.get_logger().info(f"▶️ JTRAJ RÁPIDO | Q0={q_start} -> Qf={q_stop} | T={duration:.2f}s")

        q_mat, qd_mat = self._generate_jtraj(q_start, q_stop, duration)
        self._start_trajectory(q_mat, qd_mat, duration)

    def _ctraj_callback(self, msg: Point):
        """Callback exclusivo de la Pestaña 4 (Visión Estática / ctraj estándar)"""
        if self.is_estop_active: return
        target_xyz = [msg.x, msg.y, msg.z]
        q_start = list(self.current_q)

        q_stop, reachable = inverse_kinematics(target_xyz[0], target_xyz[1], target_xyz[2], raw_float=True)
        if not reachable:
            self.get_logger().error(f"⚠️ Punto cartesiano objetivo XYZ={target_xyz} mm INALCANZABLE.")
            self._publish_status(-1)
            return

        duration = self._calculate_dynamic_duration(q_start, q_stop)
        self.get_logger().info(f"▶️ CTRAJ ESTÁTICO 3D | Meta XYZ={target_xyz} mm | T={duration:.2f}s")

        q_mat, qd_mat, success = self._generate_ctraj(q_start, target_xyz, duration)
        if success:
            self._start_trajectory(q_mat, qd_mat, duration)
        else:
            self._publish_status(-1)

    def _sync_ctraj_callback(self, msg: Point):
        """Callback exclusivo de la Pestaña 5 (Interfaz Dinámica Síncrona)"""
        if self.is_estop_active: return
        
        t_llegada = msg.x
        target_xyz = [220.0, msg.y, msg.z]  
        q_start = list(self.current_q)

        q_stop, reachable = inverse_kinematics(target_xyz[0], target_xyz[1], target_xyz[2], raw_float=True)
        if not reachable:
            self.get_logger().error(f"⚠️ Punto síncrono XYZ={target_xyz} mm INALCANZABLE.")
            self._publish_status(-1)
            return

        t_min_mecanico = self._calculate_dynamic_duration(q_start, q_stop)

        if t_llegada < t_min_mecanico:
            duration = t_min_mecanico
        else:
            duration = t_llegada

        self.get_logger().info(f"▶️ INTERCEPCIÓN SÍNCRONA | Meta XYZ={target_xyz} mm | T={duration:.2f}s")

        q_mat, qd_mat, success = self._generate_ctraj(q_start, target_xyz, duration)
        if success:
            self._start_trajectory(q_mat, qd_mat, duration)
        else:
            self._publish_status(-1)

    def _start_trajectory(self, q_mat, qd_mat, duration):
        self.trajectory_q = q_mat
        self.trajectory_qd = qd_mat
        self.total_duration = duration
        self.current_step = 0
        self.is_executing = True
        self._publish_status(1)

    def _abort_trajectory(self):
        self.is_executing = False
        self.trajectory_q = []
        self.trajectory_qd = []
        self.current_step = 0
        self._publish_status(-1)

    def _publish_status(self, code: int):
        msg = Int8()
        msg.data = code
        self.status_pub.publish(msg)

    def _control_loop(self):
        if not self.is_executing or self.is_estop_active:
            return

        total_steps = len(self.trajectory_q)

        if self.current_step < total_steps:
            q_p = self.trajectory_q[self.current_step]
            qd_p = self.trajectory_qd[self.current_step]

            traj_state = 1 if self.current_step == 0 else 2

            msg = Trama()
            msg.q = [float(q_p[0]), float(q_p[1]), float(q_p[2])]
            msg.qd = [float(qd_p[0]), float(qd_p[1]), float(qd_p[2])]
            msg.t_total = float(self.total_duration)
            msg.n_iter = int(self.current_step)
            msg.traj_state = int(traj_state)

            self.cmd_pub.publish(msg)
            self.current_step += 1
        else:
            last_q = self.trajectory_q[-1] if len(self.trajectory_q) > 0 else self.current_q
            
            final_msg = Trama()
            final_msg.q = [float(last_q[0]), float(last_q[1]), float(last_q[2])]
            final_msg.qd = [0.0, 0.0, 0.0]
            final_msg.t_total = float(self.total_duration)
            final_msg.n_iter = int(total_steps)
            final_msg.traj_state = 3

            self.cmd_pub.publish(final_msg)
            self.is_executing = False
            self.trajectory_q = []
            self.trajectory_qd = []
            self._publish_status(2)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()