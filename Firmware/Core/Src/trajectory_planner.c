/* Src/trajectory_planner.c */
#include "trajectory_planner.h"
#include "kinematics.h"
#include "motors.h"
#include <string.h>
#include <math.h>

#define STEPS_PER_REV_LOCAL   (200.0f * 8.0f)
#define DEG_TO_STEPS_LOCAL    (STEPS_PER_REV_LOCAL / 360.0f)

TrajectoryPlanner_t g_traj_planner;

void TrajectoryPlanner_Init(void) {
    memset(&g_traj_planner, 0, sizeof(TrajectoryPlanner_t));
    g_traj_planner.is_active = false;
}

bool TrajectoryPlanner_StartCtraj(const float target_xyz_mm[3], float duration_sec, const float current_q_deg[3]) {
    if (!Kinematics_IK(target_xyz_mm, g_traj_planner.q_target)) {
        return false;
    }

    if (!Kinematics_FK(current_q_deg, g_traj_planner.p_start)) {
        return false;
    }

    g_traj_planner.p_target[0] = target_xyz_mm[0];
    g_traj_planner.p_target[1] = target_xyz_mm[1];
    g_traj_planner.p_target[2] = target_xyz_mm[2];

    // 1. Calculo de deltas articulares
    float dq[3];
    dq[0] = fabsf(g_traj_planner.q_target[0] - current_q_deg[0]);
    dq[1] = fabsf(g_traj_planner.q_target[1] - current_q_deg[1]);
    dq[2] = fabsf(g_traj_planner.q_target[2] - current_q_deg[2]);

    // 2. Reducciones mecanicas
    float gear_ratio[3] = {
        108.0f / 19.0f, // Base
        32.0f / 12.0f,  // Hombro
        32.0f / 12.0f   // Codo
    };

    // 3. Frecuencia segura (85% del maximo)
    float max_freq_safe = (float)MAX_V_HZ * 0.85f;
    float t_min = 0.6f;

    // 4. Calculo del tiempo minimo para evitar saturacion de velocidad
    for (int i = 0; i < 3; i++) {
        if (dq[i] > 0.01f) {
            float t_req = (1.875f * dq[i] * DEG_TO_STEPS_LOCAL * gear_ratio[i]) / max_freq_safe;
            if (t_req > t_min) {
                t_min = t_req;
            }
        }
    }

    if (duration_sec < t_min) {
        g_traj_planner.duration = t_min;
    } else {
        g_traj_planner.duration = duration_sec;
    }

    g_traj_planner.elapsed_time = 0.0f;
    g_traj_planner.settling_time = 0.0f;

    g_traj_planner.q_last[0] = current_q_deg[0];
    g_traj_planner.q_last[1] = current_q_deg[1];
    g_traj_planner.q_last[2] = current_q_deg[2];

    g_traj_planner.is_active = true;
    return true;
}

bool TrajectoryPlanner_Update(float dt_sec, float q_cmd_out[3], float qd_cmd_out[3], const float current_q_deg[3]) {
    if (!g_traj_planner.is_active || dt_sec <= 0.0001f) {
        return false;
    }

    g_traj_planner.elapsed_time += dt_sec;
    float tau = g_traj_planner.elapsed_time / g_traj_planner.duration;

    if (tau < 1.0f) {
        // FASE 1: Trayectoria Cartesiana Quintica C2
        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float tau4 = tau3 * tau;
        float tau5 = tau4 * tau;
        float s = (10.0f * tau3) - (15.0f * tau4) + (6.0f * tau5);

        float p_current[3];
        for (int i = 0; i < 3; i++) {
            p_current[i] = g_traj_planner.p_start[i] + s * (g_traj_planner.p_target[i] - g_traj_planner.p_start[i]);
        }

        float q_current[3];
        if (!Kinematics_IK(p_current, q_current)) {
            TrajectoryPlanner_Abort();
            return false;
        }

        for (int i = 0; i < 3; i++) {
            q_cmd_out[i] = q_current[i];
            qd_cmd_out[i] = (q_current[i] - g_traj_planner.q_last[i]) / dt_sec;
            g_traj_planner.q_last[i] = q_current[i];
        }

        return true;

    } else {
        // FASE 2: Asentamiento Suave en el Objetivo Final
        for (int i = 0; i < 3; i++) {
            q_cmd_out[i] = g_traj_planner.q_target[i];
            qd_cmd_out[i] = 0.0f;
        }

        g_traj_planner.settling_time += dt_sec;

        // Comprobar si las 3 articulaciones alcanzaron la meta (tol 0.45°)
        bool all_reached = true;
        for (int i = 0; i < 3; i++) {
            if (fabsf(g_traj_planner.q_target[i] - current_q_deg[i]) > 0.45f) {
                all_reached = false;
                break;
            }
        }

        // Finalizar si todos los ejes llegaron o expiro el tiempo maximo de asentamiento (0.8s)
        if (all_reached || g_traj_planner.settling_time >= 0.8f) {
            g_traj_planner.is_active = false;
            return false;
        }

        return true;
    }
}

void TrajectoryPlanner_Abort(void) {
    g_traj_planner.is_active = false;
}
