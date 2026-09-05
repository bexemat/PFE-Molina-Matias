/**
 * @file trajectory_planner.h
 * @brief Generador analítico de perfiles de trayectoria quínticos (C^2) en espacio articular.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Proporciona interpolación continua de quinto grado con aceleración y velocidad
 * iniciales y finales nulas, garantizando transiciones suaves sin discontinuidades ni sacudidas
 * (jerk infinito). Incorpora control PD en la fase terminal para absorber la fricción mecánica.
 */

#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * PARÁMETROS OPERATIVOS Y LÍMITES TEMPORALES DEL PLANIFICADOR
 * ========================================================================= */
#define TRAJ_MIN_DURATION_S       (0.60f)   /**< Cota inferior temporal para evitar saturación dinámica [s] */
#define TRAJ_SETTLING_TIMEOUT_S   (0.80f)   /**< Tiempo máximo para fase terminal de asentamiento PD [s] */
#define TRAJ_CUTOFF_FREQ_HZ       (1.00f)   /**< Umbral mínimo de frecuencia PWM antes de detener el pulso [Hz] */
#define TRAJ_MAX_SAFE_FREQ_HZ     (850.0f)  /**< Margen seguro de frecuencia de pulsos (85% del límite físico) [Hz] */

/**
 * @brief Descriptor de estado del planificador cartesiano/articular.
 */
typedef struct {
    float p_start[3];       /**< Posición cartesiana de inicio [X, Y, Z] en [mm] */
    float p_target[3];      /**< Posición cartesiana objetivo [X, Y, Z] en [mm] */
    float q_target[3];      /**< Meta articular calculada por cinemática inversa [°] */
    float q_last[3];        /**< Última posición articular de consigna evaluada [°] */
    float duration;         /**< Duración programada del perfil quíntico [s] */
    float elapsed_time;     /**< Tiempo transcurrido en el perfil de movimiento actual [s] */
    float settling_time;    /**< Tiempo acumulado en la ventana de asentamiento terminal [s] */
    bool is_active;         /**< Estado activo de interpolación de trayectoria */
} TrajectoryPlanner_t;

extern TrajectoryPlanner_t g_traj_planner;

/**
 * @brief Inicializa los valores y estados por defecto del módulo planificador.
 */
void TrajectoryPlanner_Init(void);

/**
 * @brief Resuelve la cinemática inversa y da inicio a una nueva trayectoria quíntica.
 *
 * @param[in] target_xyz_mm  Coordenadas cartesianas de llegada [X, Y, Z] en [mm].
 * @param[in] duration_sec   Duración temporal solicitada para el movimiento [s].
 * @param[in] current_q_deg  Posición angular instantánea de los motores [°].
 *
 * @return true si la trayectoria fue validada e iniciada exitosamente; false si fue rechazada.
 */
bool TrajectoryPlanner_StartCtraj(const float target_xyz_mm[3], float duration_sec, const float current_q_deg[3]);

/**
 * @brief Actualiza de forma cíclica la consigna instantánea de posición y velocidad articular.
 * @note Diseñada para ser invocada determinísticamente a 100 Hz dentro de mControlTask.
 *
 * @param[in]  dt_sec         Periodo de muestreo del lazo de control (0.010 s) [s].
 * @param[out] q_cmd_out      Vector de consignas angulares instantáneas [°].
 * @param[out] qd_cmd_out     Vector de consignas de velocidad instantánea [°/s].
 * @param[in]  current_q_deg  Vector de posiciones angulares reales medidas [°].
 *
 * @return true mientras la trayectoria esté en progreso; false al finalizar el asentamiento.
 */
bool TrajectoryPlanner_Update(float dt_sec, float q_cmd_out[3], float qd_cmd_out[3], const float current_q_deg[3]);

/**
 * @brief Cancela de manera inmediata cualquier trayectoria activa y resetea las consignas.
 */
void TrajectoryPlanner_Abort(void);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_PLANNER_H_ */
