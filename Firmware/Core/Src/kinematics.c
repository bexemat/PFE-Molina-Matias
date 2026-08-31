/* Src/kinematics.c */
#include "kinematics.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

bool Kinematics_CheckJointLimits(const float q_deg[3]) {
    if (q_deg[0] < Q1_MIN_DEG || q_deg[0] > Q1_MAX_DEG) return false;
    if (q_deg[1] < Q2_MIN_DEG || q_deg[1] > Q2_MAX_DEG) return false;
    if (q_deg[2] < Q3_MIN_DEG || q_deg[2] > Q3_MAX_DEG) return false;
    return true;
}

bool Kinematics_FK(const float q_deg[3], float pos_mm[3]) {
    float q1_rad = q_deg[0] * DEG_TO_RAD_F;
    float q2_rad = q_deg[1] * DEG_TO_RAD_F;
    float q3_rad = q_deg[2] * DEG_TO_RAD_F;

    // Radio proyectado con offset integrado
    float R = KIN_L2 * cosf(q2_rad) + KIN_L3 * cosf(q3_rad) + KIN_O_R;

    // Coordenadas cartesianas en metros
    float x_m = R * cosf(q1_rad);
    float y_m = R * sinf(q1_rad);
    float z_m = KIN_L1 + KIN_L2 * sinf(q2_rad) + KIN_L3 * sinf(q3_rad) - KIN_O_Z;

    pos_mm[0] = x_m * 1000.0f;
    pos_mm[1] = y_m * 1000.0f;
    pos_mm[2] = z_m * 1000.0f;

    return true;
}

bool Kinematics_IK(const float pos_mm[3], float q_deg[3]) {
    float x = pos_mm[0] * 0.001f;
    float y = pos_mm[1] * 0.001f;
    float z = pos_mm[2] * 0.001f;

    // 1. Ángulo de base q1
    float q1_rad = atan2f(y, x);
    float R_obj = sqrtf(x * x + y * y);

    // Protección: singularidad cilíndrica central
    if (R_obj <= (KIN_O_R + 0.005f)) return false;

    // 2. Desplazamiento inverso al nodo de la muñeca
    float r_muneca = R_obj - KIN_O_R;
    float z_prime = z + KIN_O_Z - KIN_L1;

    // 3. Distancia D (Hombro a Muñeca)
    float D = sqrtf(r_muneca * r_muneca + z_prime * z_prime);

    // Verificación de alcanzabilidad
    if (D > (KIN_L2 + KIN_L3) || D < fabsf(KIN_L2 - KIN_L3)) return false;

    // 4. Cinemática del triángulo 2D (Ley de Cosenos)
    float cos_beta = (D * D + KIN_L2 * KIN_L2 - KIN_L3 * KIN_L3) / (2.0f * D * KIN_L2);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);

    float alpha = atan2f(z_prime, r_muneca);

    float cos_gamma = (D * D + KIN_L3 * KIN_L3 - KIN_L2 * KIN_L2) / (2.0f * D * KIN_L3);
    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    float gamma = acosf(cos_gamma);

    // 5. Ángulos absolutos
    float q2_rad = alpha + beta;
    float q3_rad = alpha - gamma;

    // Conversión a grados
    q_deg[0] = q1_rad * RAD_TO_DEG;
    q_deg[1] = q2_rad * RAD_TO_DEG;
    q_deg[2] = q3_rad * RAD_TO_DEG;

    // Verificación de límites mecánicos
    return Kinematics_CheckJointLimits(q_deg);
}
