/**
 * @file kinematics.c
 * @brief Implementación analítica del modelado cinemático directo e inverso.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Este módulo resuelve el mapeo cinemático para el manipulador paralelo de 3 GDL
 * (arquitectura EEZYbotARM MK2 modificada). Utiliza una formulación trigonométrica cerrada
 * en el plano proyectado r-z' generada a partir de los parámetros constructivos validados en CAD.
 *
 * @note Principios matemáticos implementados:
 *  - Eje 1 (Cintura): Rotación azimutal cartesiana en el plano X-Y.
 *  - Ejes 2 y 3 (Hombro y Muñeca): Desacople en el plano sagital r-z' mediante la Ley del Coseno.
 *  - Eliminación de articulaciones virtuales D-H: El paralelismo inducido por el mecanismo de
 *    cuatro barras se resuelve de forma directa manteniendo la orientación horizontal del efector.
 */

#include "kinematics.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F (3.14159265358979323846f)
#endif

/**
 * @brief Evalúa si un vector de configuración articular respeta los topes mecánicos del robot.
 *
 * @param[in] q_deg Arreglo de 3 elementos con las coordenadas articulares [Q1, Q2, Q3] en grados [°].
 *
 * @return true  Las 3 articulaciones están estrictamente dentro de sus cotas operativas.
 * @return false Al menos una articulación excede su límite admisible.
 *
 * @note Límites configurados:
 *       - Q1: [-90.0°, +90.0°]
 *       - Q2: [+40.0°, +157.0°]
 *       - Q3: [-75.0°, +25.0°]
 */
bool Kinematics_CheckJointLimits(const float q_deg[3]) {
    if ((q_deg[0] < Q1_MIN_DEG) || (q_deg[0] > Q1_MAX_DEG)) {
        return false;
    }
    if ((q_deg[1] < Q2_MIN_DEG) || (q_deg[1] > Q2_MAX_DEG)) {
        return false;
    }
    if ((q_deg[2] < Q3_MIN_DEG) || (q_deg[2] > Q3_MAX_DEG)) {
        return false;
    }
    return true;
}

/**
 * @brief Resuelve la cinemática directa analítica (Forward Kinematics).
 *
 * @details Transforma las coordenadas generalizadas articulares [Q1, Q2, Q3] a la posición
 * tridimensional [X, Y, Z] del centro del efector final (solenoide) referida a la base.
 *
 * Ecuaciones:
 * \f[
 *   R = L_2 \cos(Q_2) + L_3 \cos(Q_3) + O_r
 * \f]
 * \f[
 *   X = R \cos(Q_1), \quad Y = R \sin(Q_1)
 * \f]
 * \f[
 *   Z = L_1 + L_2 \sin(Q_2) + L_3 \sin(Q_3) - O_z
 * \f]
 *
 * @param[in]  q_deg   Ángulos absolutos de las articulaciones [Q1, Q2, Q3] en grados sexagesimales [°].
 * @param[out] pos_mm  Vector cartesiano resultante [X, Y, Z] en milímetros [mm].
 *
 * @return true  Cálculo completado correctamente.
 * @note Esta función es puramente determinista y reentrante (thread-safe).
 */
bool Kinematics_FK(const float q_deg[3], float pos_mm[3]) {
    /* Conversión de unidades de entrada sexagesimales a radianes */
    const float q1_rad = q_deg[0] * DEG_TO_RAD_F;
    const float q2_rad = q_deg[1] * DEG_TO_RAD_F;
    const float q3_rad = q_deg[2] * DEG_TO_RAD_F;

    /* Proyección del radio planar total, integrando el offset constructivo del cabezal Or */
    const float r_planar = (KIN_L2_MM * cosf(q2_rad)) + (KIN_L3_MM * cosf(q3_rad)) + KIN_O_R_MM;

    /* Proyección cartesiana tridimensional referida al centro del eje de cintura */
    pos_mm[0] = r_planar * cosf(q1_rad);
    pos_mm[1] = r_planar * sinf(q1_rad);
    pos_mm[2] = KIN_L1_MM + (KIN_L2_MM * sinf(q2_rad)) + (KIN_L3_MM * sinf(q3_rad)) - KIN_O_Z_MM;

    return true;
}

/**
 * @brief Resuelve la cinemática inversa analítica geométrica (Inverse Kinematics).
 *
 * @details Determina la configuración articular requerida para alcanzar una posición objetivo [X, Y, Z].
 * Implementa la solución de rama geométrica "codo arriba" (elbow-up) de manera analítica y única,
 * evitando iteraciones y singularidades cinemáticas.
 *
 * Pasos algorítmicos:
 *  1. Cálculo de Q1 mediante proyección azimutal atan2(Y, X).
 *  2. Desacople de los offsets constructivos del efector (Or, Oz) para hallar el nodo de muñeca (r_wrist, z').
 *  3. Verificación de la distancia euclídea D contra el espacio de trabajo admisible (|L2 - L3| <= D <= L2 + L3).
 *  4. Cálculo de los ángulos internos beta y gamma mediante el Teorema del Coseno.
 *  5. Composición de Q2 = alpha + beta y Q3 = alpha - gamma.
 *  6. Validación final de los ángulos resultantes contra límites físicos de carrera.
 *
 * @param[in]  pos_mm  Coordenadas cartesianas deseadas [X, Y, Z] en milímetros [mm].
 * @param[out] q_deg   Arreglo de 3 elementos donde se retornan los ángulos [Q1, Q2, Q3] en grados [°].
 *
 * @return true  Pose cartesiana admisible y solución articular físicamente ejecutable.
 * @return false Punto fuera del volumen de trabajo o que viola los límites angulares.
 *
 * @warning Si el punto solicitado cae en la singularidad cilíndrica central (R_obj <= Or + 5 mm),
 *          la función aborta inmediatamente retornando false.
 */
bool Kinematics_IK(const float pos_mm[3], float q_deg[3]) {
    const float x = pos_mm[0];
    const float y = pos_mm[1];
    const float z = pos_mm[2];

    /* -------------------------------------------------------------------------
     * PASO 1: Rotación de base azimutal (Articulación Q1)
     * ------------------------------------------------------------------------- */
    const float q1_rad = atan2f(y, x);
    const float r_cyl = sqrtf((x * x) + (y * y));

    /* Aislamiento preventivo: evita singularidad de indeterminación en el eje vertical */
    if (r_cyl <= (KIN_O_R_MM + 5.0f)) {
        return false;
    }

    /* -------------------------------------------------------------------------
     * PASO 2: Traslación al centro del nodo de articulación de la muñeca
     * ------------------------------------------------------------------------- */
    const float r_wrist = r_cyl - KIN_O_R_MM;
    const float z_prime = z + KIN_O_Z_MM - KIN_L1_MM;

    /* -------------------------------------------------------------------------
     * PASO 3: Distancia euclídea D entre origen del hombro y nodo de muñeca
     * ------------------------------------------------------------------------- */
    const float d_dist = sqrtf((r_wrist * r_wrist) + (z_prime * z_prime));

    /* Validación del volumen de trabajo esférico permitido */
    if ((d_dist > (KIN_L2_MM + KIN_L3_MM)) || (d_dist < fabsf(KIN_L2_MM - KIN_L3_MM))) {
        return false;
    }

    /* -------------------------------------------------------------------------
     * PASO 4: Resolución planar 2D mediante Teorema del Coseno
     * ------------------------------------------------------------------------- */
    /* Ángulo interno beta (entre vector D y eslabón principal L2) */
    float cos_beta = ((d_dist * d_dist) + (KIN_L2_MM * KIN_L2_MM) - (KIN_L3_MM * KIN_L3_MM)) /
                     (2.0f * d_dist * KIN_L2_MM);
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    const float beta = acosf(cos_beta);

    /* Ángulo de elevación alpha del vector D respecto al horizonte */
    const float alpha = atan2f(z_prime, r_wrist);

    /* Ángulo interno gamma (entre vector D y antebrazo L3) */
    float cos_gamma = ((d_dist * d_dist) + (KIN_L3_MM * KIN_L3_MM) - (KIN_L2_MM * KIN_L2_MM)) /
                      (2.0f * d_dist * KIN_L3_MM);
    if (cos_gamma > 1.0f)  cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    const float gamma = acosf(cos_gamma);

    /* -------------------------------------------------------------------------
     * PASO 5: Composición angular absoluta de la cadena cinemática ("Codo Arriba")
     * ------------------------------------------------------------------------- */
    const float q2_rad = alpha + beta;
    const float q3_rad = alpha - gamma;

    /* Conversión de radianes a grados sexagesimales */
    q_deg[0] = q1_rad * RAD_TO_DEG;
    q_deg[1] = q2_rad * RAD_TO_DEG;
    q_deg[2] = q3_rad * RAD_TO_DEG;

    /* -------------------------------------------------------------------------
     * PASO 6: Validación de topes mecánicos físicos
     * ------------------------------------------------------------------------- */
    return Kinematics_CheckJointLimits(q_deg);
}
