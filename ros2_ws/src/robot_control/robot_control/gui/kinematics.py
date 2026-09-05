"""Modelado cinemático analítico para el manipulador paralelo de 3 GDL.

Implementa la cinemática directa (FK) e inversa (IK) geométrica en el
plano sagital proyectado r-z', eliminando la necesidad de eslabones y
articulaciones virtuales de la convención Denavit-Hartenberg.
"""

from typing import List, Tuple
import numpy as np

# =========================================================================
# PARÁMETROS GEOMÉTRICOS Y OFFSETS CONSTRUCTIVOS [m]
# =========================================================================
L1_M: float = 0.07700   # Altura axial de la base al centro del hombro [m]
L2_M: float = 0.13500   # Eslabón principal de elevación (hombro a codo) [m]
L3_M: float = 0.14700   # Eslabón de antebrazo (codo a muñeca) [m]
O_R_M: float = 0.02549  # Desfase radial constructivo del efector final [m]
O_Z_M: float = 0.04087  # Desfase axial vertical del efector final [m]

# Límites angulares de seguridad física articular [°]
Q1_MIN_DEG: float = -90.0
Q1_MAX_DEG: float = 90.0
Q2_MIN_DEG: float = 40.0   # Mantiene coherencia estricta con el firmware
Q2_MAX_DEG: float = 157.0
Q3_MIN_DEG: float = -75.0
Q3_MAX_DEG: float = 25.0


def forward_kinematics(
    q_deg: List[float],
) -> Tuple[float, float, float, np.ndarray]:
    """Resuelve la cinemática directa analítica (FK).

    Calcula la posición cartesiana [X, Y, Z] del efector final referida al
    origen de la base a partir del vector de ángulos articulares.

    Args:
        q_deg (List[float]): Vector articular [Q1, Q2, Q3] en grados [°].

    Returns:
        Tuple[float, float, float, np.ndarray]:
            - x_mm (float): Coordenada longitudinal X [mm].
            - y_mm (float): Coordenada transversal Y [mm].
            - z_mm (float): Coordenada vertical Z [mm].
            - T_total (np.ndarray): Matriz de transformación homogénea 4x4 [m].
    """
    q_rad = np.radians(np.round(q_deg, 4))
    q1, q2 = float(q_rad[0]), float(q_rad[1])
    q3 = float(q_rad[2]) if len(q_rad) > 2 else 0.0

    # Radio proyectado planar integrando desfase del cabezal Or
    radius_m = L2_M * np.cos(q2) + L3_M * np.cos(q3) + O_R_M

    # Posición tridimensional referida a la base en metros
    x_m = radius_m * np.cos(q1)
    y_m = radius_m * np.sin(q1)
    z_m = L1_M + L2_M * np.sin(q2) + L3_M * np.sin(q3) - O_Z_M

    # Conversión métrica a milímetros
    x_mm = float(x_m * 1000.0)
    y_mm = float(y_m * 1000.0)
    z_mm = float(z_m * 1000.0)

    if abs(y_mm) < 1e-4:
        y_mm = 0.0

    # Construcción de la matriz homogénea cartesiana simplificada
    ct1, st1 = np.cos(q1), np.sin(q1)
    t_total = np.array(
        [
            [ct1, -st1, 0.0, x_m],
            [st1, ct1, 0.0, y_m],
            [0.0, 0.0, 1.0, z_m],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    return x_mm, y_mm, z_mm, t_total


def inverse_kinematics(
    x_mm: float, y_mm: float, z_mm: float, raw_float: bool = True
) -> Tuple[List[float], bool]:
    """Resuelve la cinemática inversa analítica geométrica (IK).

    Determina la solución articular para alcanzar el objetivo cartesiano,
    forzando la configuración geométrica de codo arriba ("elbow-up").

    Args:
        x_mm (float): Coordenada objetivo X en milímetros [mm].
        y_mm (float): Coordenada objetivo Y en milímetros [mm].
        z_mm (float): Coordenada objetivo Z en milímetros [mm].
        raw_float (bool, optional): Si es True retorna flotantes continuos;
            si es False redondea a dos decimales. Por defecto es True.

    Returns:
        Tuple[List[float], bool]:
            - q_deg (List[float]): Vector articular resuelto [Q1, Q2, Q3] en grados [°].
            - reachable (bool): True si la solución es alcanzable y físicamente válida;
                                False si cae fuera del workspace o viola límites.
    """
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0

    # 1. Rotación azimutal de la base (Articulación Q1)
    q1_rad = np.arctan2(y, x)
    r_obj = np.hypot(x, y)

    # Protección contra indeterminación en eje central
    if r_obj <= (O_R_M + 0.005):
        return [0.0, 0.0, 0.0], False

    # 2. Desacople geométrico hacia el nodo de articulación de muñeca
    r_wrist = r_obj - O_R_M
    z_prime = z + O_Z_M - L1_M

    # 3. Distancia euclídea planar entre hombro y muñeca
    d_dist = np.hypot(r_wrist, z_prime)

    # Comprobación de alcanzabilidad en volumen esférico
    if d_dist > (L2_M + L3_M) or d_dist < abs(L2_M - L3_M):
        return [0.0, 0.0, 0.0], False

    # 4. Solución planar 2D mediante Teorema del Coseno
    cos_beta = np.clip(
        (d_dist**2 + L2_M**2 - L3_M**2) / (2.0 * d_dist * L2_M), -1.0, 1.0
    )
    beta = np.arccos(cos_beta)

    alpha = np.arctan2(z_prime, r_wrist)

    cos_gamma = np.clip(
        (d_dist**2 + L3_M**2 - L2_M**2) / (2.0 * d_dist * L3_M), -1.0, 1.0
    )
    gamma = np.arccos(cos_gamma)

    # 5. Ángulos absolutos en configuración codo arriba
    q2_rad = alpha + beta
    q3_rad = alpha - gamma

    q1_deg = float(np.degrees(q1_rad))
    q2_deg = float(np.degrees(q2_rad))
    q3_deg = float(np.degrees(q3_rad))

    # 6. Validación contra límites de carrera articulares
    if not (Q1_MIN_DEG <= q1_deg <= Q1_MAX_DEG):
        return [0.0, 0.0, 0.0], False
    if not (Q2_MIN_DEG <= q2_deg <= Q2_MAX_DEG):
        return [0.0, 0.0, 0.0], False
    if not (Q3_MIN_DEG <= q3_deg <= Q3_MAX_DEG):
        return [0.0, 0.0, 0.0], False

    if raw_float:
        return [q1_deg, q2_deg, q3_deg], True
    return [round(q1_deg, 2), round(q2_deg, 2), round(q3_deg, 2)], True