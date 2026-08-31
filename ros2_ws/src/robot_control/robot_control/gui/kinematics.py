import numpy as np

# =========================================================================
# PARÁMETROS GEOMÉTRICOS VALIDADOS EN SOLIDWORKS (en metros)
# =========================================================================
L1 = 0.07700   # Altura base-hombro (77.00 mm)
L2 = 0.13500   # Longitud del brazo principal (135.00 mm)
L3 = 0.14700   # Longitud del antebrazo (147.00 mm)
O_r = 0.02549  # Offset radial del efector (25.49 mm)
O_z = 0.04087  # Offset vertical del efector (40.87 mm)

def forward_kinematics(q_deg):
    """
    Cinemática Directa Trigonométrica (FK).
    q1: Rotación de la base
    q2: Elevación absoluta del brazo (L2) respecto a la horizontal
    q3: Inclinación absoluta del antebrazo (L3) respecto a la horizontal
    """
    q_rad = np.radians(np.round(q_deg, 4))
    q1, q2 = q_rad[0], q_rad[1]
    q3 = q_rad[2] if len(q_rad) > 2 else 0.0

    # Radio y altura con offsets integrados
    R = L2 * np.cos(q2) + L3 * np.cos(q3) + O_r
    
    x_m = R * np.cos(q1)
    y_m = R * np.sin(q1)
    z_m = L1 + L2 * np.sin(q2) + L3 * np.sin(q3) - O_z

    x_mm = x_m * 1000.0
    y_mm = y_m * 1000.0
    z_mm = z_m * 1000.0

    if abs(y_mm) < 1e-4: 
        y_mm = 0.0

    # Matriz T_total simplificada para la posición del TCP
    ct1, st1 = np.cos(q1), np.sin(q1)
    T_total = np.array([
        [ct1, -st1, 0.0, x_m],
        [st1,  ct1, 0.0, y_m],
        [0.0,  0.0, 1.0, z_m],
        [0.0,  0.0, 0.0, 1.0]
    ], dtype=np.float64)

    return x_mm, y_mm, z_mm, T_total

def inverse_kinematics(x_mm, y_mm, z_mm, raw_float=True):
    """
    Cinemática Inversa Geométrica (IK).
    """
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0

    # 1. Ángulo de la base
    q1_rad = np.arctan2(y, x)
    R_obj = np.hypot(x, y)
    
    # PROTECCIÓN: Evitar singularidad cerca del eje central
    if R_obj <= (O_r + 0.005): 
        return [0.0, 0.0, 0.0], False

    # 2. Desplazamiento inverso al nodo de la muñeca
    r_muneca = R_obj - O_r
    z_prime = z + O_z - L1

    # 3. Distancia D (Hombro a Muñeca)
    D = np.hypot(r_muneca, z_prime)

    # Verificación de alcanzabilidad
    if D > (L2 + L3) or D < abs(L2 - L3):
        return [0.0, 0.0, 0.0], False

    # 4. Cinemática del triángulo 2D (Ley de Cosenos)
    # Beta: ángulo interno entre L2 y D
    cos_beta = np.clip((D**2 + L2**2 - L3**2) / (2.0 * D * L2), -1.0, 1.0)
    beta = np.arccos(cos_beta)
    
    # Alfa: ángulo de D respecto a la horizontal
    alpha = np.arctan2(z_prime, r_muneca)
    
    # Gamma: ángulo interno entre D y L3
    cos_gamma = np.clip((D**2 + L3**2 - L2**2) / (2.0 * D * L3), -1.0, 1.0)
    gamma = np.arccos(cos_gamma)

    # 5. Ángulos absolutos resultantes
    q2_rad = alpha + beta
    q3_rad = alpha - gamma

    q1_deg = float(np.degrees(q1_rad))
    q2_deg = float(np.degrees(q2_rad))
    q3_deg = float(np.degrees(q3_rad))

    # PROTECCIÓN: Límites articulares
    if not (-90.0 <= q1_deg <= 90.0): return [0.0, 0.0, 0.0], False
    if not (30.0 <= q2_deg <= 157.0): return [0.0, 0.0, 0.0], False
    if not (-75.0 <= q3_deg <= 25.0): return [0.0, 0.0, 0.0], False

    if raw_float:
        return [q1_deg, q2_deg, q3_deg], True
    return [round(q1_deg, 2), round(q2_deg, 2), round(q3_deg, 2)], True