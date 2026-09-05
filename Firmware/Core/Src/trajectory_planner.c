/**
 * @file trajectory_planner.c
 * @brief Generador analítico de perfiles de interpolación quínticos y lazo de asentamiento PD.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Este módulo implementa un planificador analítico polinómico de quinto grado (clase C^2).
 * Proporciona transiciones continuas con velocidad y aceleración nulas en ambos extremos:
 * \f[
 *   s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau = \frac{t}{T} \in [0, 1]
 * \f]
 * Esto garantiza jerk finito, eliminando discontinuidades de inercia y vibraciones en piezas impresas en 3D.
 * Incluye saturación dinámica de velocidad por eje y transición a un lazo de asentamiento terminal.
 */

#include "trajectory_planner.h"
#include "kinematics.h"
#include "motors.h"
#include <string.h>
#include <math.h>

/* Parámetros de conversión para actuadores paso a paso configurados a 1/8 micropaso */
#define STEPS_PER_REV_LOCAL   (200.0f * 8.0f)
#define DEG_TO_STEPS_LOCAL    (STEPS_PER_REV_LOCAL / 360.0f)

/** @brief Instancia global del estado del planificador cartesiano/articular */
TrajectoryPlanner_t g_traj_planner;

/**
 * @brief Inicializa a cero las variables de estado y desactiva el planificador.
 */
void TrajectoryPlanner_Init(void) {
    memset(&g_traj_planner, 0, sizeof(TrajectoryPlanner_t));
    g_traj_planner.is_active = false;
}

/**
 * @brief Planifica y valida una nueva trayectoria cartesiana continua.
 *
 * @details Realiza los siguientes pasos de control determinista:
 *  1. Valida alcanzabilidad del objetivo resolviendo la cinemática inversa final (q_target).
 *  2. Estima la posición cartesiana inicial (p_start) a partir de la telemetría actual.
 *  3. Calcula el tiempo mínimo viable (t_min) analizando el desplazamiento angular por eje,
 *     la relación de reducción y el pico de aceleración quíntica (factor 1.875), limitando
 *     la frecuencia máxima al 85% de la velocidad límite del motor (850 Hz).
 *  4. Configura la duración definitiva asegurando que duration >= max(TRAJ_MIN_DURATION_S, t_min).
 *
 * @param[in] target_xyz_mm Coordenadas de llegada deseadas [X, Y, Z] en milímetros [mm].
 * @param[in] duration_sec  Tiempo de traslado solicitado desde el supervisor ROS 2 [s].
 * @param[in] current_q_deg Ángulos articulares actuales medidos por los encoders [°].
 *
 * @return true  Trayectoria admitida y parámetros cargados en la estructura global.
 * @return false Rechazo cinemático por objetivo fuera de workspace o singularidad.
 */
bool TrajectoryPlanner_StartCtraj(const float target_xyz_mm[3], float duration_sec, const float current_q_deg[3]) {
    /* Validación cinemática del punto de destino */
    if (!Kinematics_IK(target_xyz_mm, g_traj_planner.q_target)) {
        return false;
    }

    /* Estimación cartesiana de origen basada en los ángulos reales del manipulador */
    if (!Kinematics_FK(current_q_deg, g_traj_planner.p_start)) {
        return false;
    }

    /* Almacenamiento de puntos extremos en el espacio tridimensional */
    g_traj_planner.p_target[0] = target_xyz_mm[0];
    g_traj_planner.p_target[1] = target_xyz_mm[1];
    g_traj_planner.p_target[2] = target_xyz_mm[2];

    /* 1. Desplazamiento angular demandado en cada articulación [°] */
    float dq[3];
    dq[0] = fabsf(g_traj_planner.q_target[0] - current_q_deg[0]);
    dq[1] = fabsf(g_traj_planner.q_target[1] - current_q_deg[1]);
    dq[2] = fabsf(g_traj_planner.q_target[2] - current_q_deg[2]);

    /* 2. Relaciones de reducción mecánica de los trenes de engranajes */
    const float gear_ratio[3] = {
        108.0f / 19.0f, /* Eje 1 (Cintura) */
        32.0f / 12.0f,  /* Eje 2 (Hombro) */
        32.0f / 12.0f   /* Eje 3 (Muñeca) */
    };

    /* 3. Cálculo del tiempo mínimo admisible para no superar el 85% de frecuencia límite */
    const float max_freq_safe = TRAJ_MAX_SAFE_FREQ_HZ;
    float t_min = TRAJ_MIN_DURATION_S;

    for (int i = 0; i < 3; i++) {
        if (dq[i] > 0.01f) {
            /* Factor 1.875 surge de la derivada máxima del polinomio quíntico C^2 normalizado */
            const float t_req = (1.875f * dq[i] * DEG_TO_STEPS_LOCAL * gear_ratio[i]) / max_freq_safe;
            if (t_req > t_min) {
                t_min = t_req;
            }
        }
    }

    /* 4. Asignación temporal con guarda de seguridad */
    g_traj_planner.duration = (duration_sec < t_min) ? t_min : duration_sec;
    g_traj_planner.elapsed_time = 0.0f;
    g_traj_planner.settling_time = 0.0f;

    /* Inicialización de referencia de velocidad articular por derivación finita */
    g_traj_planner.q_last[0] = current_q_deg[0];
    g_traj_planner.q_last[1] = current_q_deg[1];
    g_traj_planner.q_last[2] = current_q_deg[2];

    g_traj_planner.is_active = true;
    return true;
}

/**
 * @brief Actualiza cíclicamente el perfil polinómico a 100 Hz e interpola consignas.
 *
 * @details El algoritmo conmuta en dos fases operativas:
 *  - **Fase 1 (tau < 1.0): Interpolación Quíntica C^2:**
 *    Calcula el parámetro normalizado tau = t / T, evalúa la coordenada cartesiana instantánea,
 *    resuelve la cinemática inversa y deriva la consigna de velocidad feedforward qd_cmd.
 *  - **Fase 2 (tau >= 1.0): Asentamiento Terminal y Control Fino:**
 *    Congela la meta angular en q_target y supervisa que el error residual en los 3 ejes sea
 *    inferior a 0.45°. Si se cumple la tolerancia o transcurren 0.8 s (timeout de fricción),
 *    da por finalizada la trayectoria de forma segura.
 *
 * @param[in]  dt_sec         Paso temporal del lazo de control (0.010 s) [s].
 * @param[out] q_cmd_out      Vector de consignas angulares instantáneas calculadas [°].
 * @param[out] qd_cmd_out     Vector de consignas de velocidad instantánea calculadas [°/s].
 * @param[in]  current_q_deg  Vector de posiciones reales actuales leídas por encoders [°].
 *
 * @return true  Trayectoria en ejecución activa.
 * @return false Trayectoria concluida (asentamiento completado o finalizado por timeout).
 */
bool TrajectoryPlanner_Update(float dt_sec, float q_cmd_out[3], float qd_cmd_out[3], const float current_q_deg[3]) {
    if (!g_traj_planner.is_active || (dt_sec <= 0.0001f)) {
        return false;
    }

    g_traj_planner.elapsed_time += dt_sec;
    const float tau = g_traj_planner.elapsed_time / g_traj_planner.duration;

    if (tau < 1.0f) {
        /* =====================================================================
         * FASE 1: Trayectoria Cartesiana Quíntica C^2 Continua
         * ===================================================================== */
        const float tau2 = tau * tau;
        const float tau3 = tau2 * tau;
        const float tau4 = tau3 * tau;
        const float tau5 = tau4 * tau;
        const float s = (10.0f * tau3) - (15.0f * tau4) + (6.0f * tau5);

        /* Interpolación lineal del vector posición cartesiana */
        float p_current[3];
        for (int i = 0; i < 3; i++) {
            p_current[i] = g_traj_planner.p_start[i] + s * (g_traj_planner.p_target[i] - g_traj_planner.p_start[i]);
        }

        /* Transformación inversa analítica a coordenadas generalizadas */
        float q_current[3];
        if (!Kinematics_IK(p_current, q_current)) {
            TrajectoryPlanner_Abort();
            return false;
        }

        /* Cálculo de velocidades feedforward por derivada discreta en el paso dt */
        for (int i = 0; i < 3; i++) {
            q_cmd_out[i] = q_current[i];
            qd_cmd_out[i] = (q_current[i] - g_traj_planner.q_last[i]) / dt_sec;
            g_traj_planner.q_last[i] = q_current[i];
        }

        return true;

    } else {
        /* =====================================================================
         * FASE 2: Asentamiento Terminal Estático con Lazo Cerrado PD
         * ===================================================================== */
        for (int i = 0; i < 3; i++) {
            q_cmd_out[i] = g_traj_planner.q_target[i];
            qd_cmd_out[i] = 0.0f;
        }

        g_traj_planner.settling_time += dt_sec;

        /* Comprobación de convergencia en todas las articulaciones (tolerancia: 0.45°) */
        bool all_reached = true;
        for (int i = 0; i < 3; i++) {
            if (fabsf(g_traj_planner.q_target[i] - current_q_deg[i]) > 0.45f) {
                all_reached = false;
                break;
            }
        }

        /* Fin de trayectoria por convergencia angular o expiración de ventana de seguridad */
        if (all_reached || (g_traj_planner.settling_time >= TRAJ_SETTLING_TIMEOUT_S)) {
            g_traj_planner.is_active = false;
            return false;
        }

        return true;
    }
}

/**
 * @brief Cancela de forma inmediata cualquier trayectoria activa en el planificador.
 */
void TrajectoryPlanner_Abort(void) {
    g_traj_planner.is_active = false;
}
