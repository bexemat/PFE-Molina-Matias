/**
 * @file trajectory_planner.h
 * @brief Generador analítico de perfiles quínticos (C^2) y lazo de asentamiento terminal.
 * @author Matías Exequiel Molina <matimolina123@gmail.com>
 * @date 2026
 *
 * @details Implementa interpolación polinómica de quinto grado con velocidad y aceleración
 * nulas en ambos extremos, garantizando derivada de aceleración finita (jerk acotado)
 * para mitigar vibraciones estructurales sobre piezas manufacturadas en 3D.
 * Incluye cálculo del tiempo mínimo admisible (T_min), fallback automático a espacio articular
 * ante singularidades intermedias y supervisión de convergencia angular terminal.
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
#define TRAJ_MIN_DURATION_S       (0.60f)   /**< Cota inferior temporal absoluta del firmware [s] */
#define TRAJ_SETTLING_TIMEOUT_S   (0.80f)   /**< Tiempo límite de guarda para asentamiento terminal [s] */
#define TRAJ_CUTOFF_FREQ_HZ       (1.00f)   /**< Umbral mínimo de frecuencia PWM antes de corte [Hz] */
#define TRAJ_MAX_SAFE_FREQ_HZ     (850.0f)  /**< 85% de la velocidad máxima admisible en motores paso a paso [Hz] */

/**
 * @brief Descriptor de estado del planificador cartesiano/articular.
 */
typedef struct {
    float p_start[3];       /**< Posición cartesiana de inicio [X, Y, Z] en [mm] */
    float p_target[3];      /**< Posición cartesiana objetivo [X, Y, Z] en [mm] */
    float q_start[3];       /**< Pose articular inicial del manipulador [°] */
    float q_target[3];      /**< Meta articular calculada por cinemática inversa [°] */
    float q_last[3];        /**< Última posición articular de consigna evaluada [°] */
    float duration;         /**< Duración programada del perfil quíntico [s] */
    float elapsed_time;     /**< Tiempo transcurrido en el perfil de movimiento actual [s] */
    float settling_time;    /**< Tiempo acumulado en la ventana de asentamiento terminal [s] */
    bool is_active;         /**< Estado activo de interpolación de trayectoria */
} TrajectoryPlanner_t;

extern TrajectoryPlanner_t g_traj_planner;

/**
 * @brief Códigos de estado del planificador propagados hacia ROS 2 (/planner/traj_status).
 */
typedef enum {
    TRAJ_STATUS_ABORT_ESTOP      = -3,  /**< Cancelación forzada por Parada de Emergencia Software */
    TRAJ_STATUS_ERROR_IK_PATH    = -2,  /**< Error dinámico durante interpolación cartesiana */
    TRAJ_STATUS_ERROR_IK_START   = -1,  /**< Meta inalcanzable al validar IK inicial en StartCtraj */
    TRAJ_STATUS_IDLE             =  0,  /**< Sin trayectoria activa / Estado de reposo */
    TRAJ_STATUS_RUNNING          =  1,  /**< En ejecución continua (interpolación quíntica o asentamiento) */
    TRAJ_STATUS_SUCCESS          =  2,  /**< Posicionamiento completado exitosamente (error articular <= 0.45°) */
    TRAJ_STATUS_TIMEOUT          =  3   /**< Finalización por expiración de guarda temporal de 0.80 s */
} TrajectoryStatus_t;

/**
 * @brief Inicializa a cero las variables de estado y desactiva el planificador.
 */
void TrajectoryPlanner_Init(void);

/**
 * @brief Valida la alcanzabilidad e inicia una nueva trayectoria cartesiana continua.
 *
 * @param[in] target_xyz_mm  Coordenadas cartesianas de llegada [X, Y, Z] en [mm].
 * @param[in] duration_sec   Duración temporal solicitada desde el supervisor ROS 2 [s].
 * @param[in] current_q_deg  Posición angular instantánea actual de los motores [°].
 *
 * @return true si la meta es alcanzable e inicia la interpolación; false si se rechaza por cinemática.
 */
bool TrajectoryPlanner_StartCtraj(const float target_xyz_mm[3], float duration_sec, const float current_q_deg[3]);

/**
 * @brief Actualiza cíclicamente el perfil polinómico a 100 Hz e interpola consignas.
 * @note Diseñada para ser invocada determinísticamente cada 10 ms (dt = 0.010 s) dentro de mControlTask.
 *
 * @details Conmuta en dos fases operativas:
 *  - **Fase 1 (tau < 1.0):** Interpola en coordenadas cartesianas, resuelve la cinemática inversa
 *    y deriva la velocidad feedforward instantánea. Si detecta pérdida de workspace, conmuta a
 *    interpolación articular quíntica pura entre q_start y q_target para asegurar arribo sin saltos.
 *  - **Fase 2 (tau >= 1.0):** Fija q_target y supervisa el asentamiento terminal. Si el error
 *    articular en los 3 ejes es <= 0.45°, retorna TRAJ_STATUS_SUCCESS (2). Si transcurren 0.80 s
 *    sin converger, finaliza retornando TRAJ_STATUS_TIMEOUT (3).
 *
 * @param[in]  dt_sec         Periodo de muestreo del lazo de control (0.010 s) [s].
 * @param[out] q_cmd_out      Vector de consignas angulares instantáneas calculadas [°].
 * @param[out] qd_cmd_out     Vector de consignas de velocidad instantánea feedforward [°/s].
 * @param[in]  current_q_deg  Vector de posiciones angulares reales medidas por los encoders [°].
 *
 * @return TrajectoryStatus_t Código de estado de ejecución para sincronismo con la FSM en ROS 2.
 */
TrajectoryStatus_t TrajectoryPlanner_Update(float dt_sec, float q_cmd_out[3], float qd_cmd_out[3], const float current_q_deg[3]);

/**
 * @brief Cancela de forma inmediata cualquier trayectoria activa y desactiva el planificador.
 */
void TrajectoryPlanner_Abort(void);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_PLANNER_H_ */
