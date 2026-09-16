/**
 * @file motors.h
 * @brief Controlador de bajo nivel para actuadores paso a paso (DRV8825) y lazo cerrado con encoders AS5600.
 * @author Matías Exequiel Molina <matimolina123@gmail.com>
 * @date 2026
 *
 * @details Gestiona la modulación PWM por hardware (TIM2, TIM3, TIM13) y pines de sentido de giro (GPIO).
 * Implementa dos esquemas de control:
 *  - Perfil trapezoidal coordinado multi-eje para movimientos punto a punto (P2P).
 *  - Lazo de seguimiento continuo con prealimentación de velocidad (Feedforward) y realimentación
 *    proporcional (P) de posición para perfiles quínticos interpolados a 100 Hz.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALL_MOTORS_IDLE ((motor1.state == IDLE) && (motor2.state == IDLE) && (motor3.state == IDLE))

/* =========================================================================
 * PARÁMETROS CINEMÁTICOS DE ACCIONAMIENTO (Modo 1/8 Micropaso)
 * ========================================================================= */
#define SPEED_P2P_DEFAULT_HZ  (350.0f)   /**< Velocidad de régimen nominal para traslados P2P [Hz] */
#define MIN_SPEED_HZ          (150.0f)   /**< Frecuencia de despegue inicial para mitigar pérdida de paso [Hz] */
#define MAX_V_HZ              (1000.0f)  /**< Frecuencia máxima absoluta admisible por el hardware [Hz] */
#define MAX_A_HZ_S            (2000.0f)  /**< Rampa máxima de aceleración lineal admisible [Hz/s] */

/**
 * @brief Estados operativos de la máquina de estados de cada eje.
 */
typedef enum {
    IDLE = 0,       /**< Actuador en reposo con bobinas excitadas estáticamente */
    P2P,            /**< Desplazamiento articular punto a punto con perfil trapezoidal */
    TRAJ,           /**< Seguimiento de trayectoria polinómica continua (Feedforward + P) */
    HOMING,         /**< Calibración y búsqueda activa de la referencia articular de origen */
    MOTOR_ERROR     /**< Estado de fallo o enclavamiento por parada de emergencia */
} MotorState_t;

/**
 * @brief Descriptor de hardware, telemetría y parámetros de control por eje.
 */
typedef struct {
    uint8_t id;                     /**< Identificador del eje (1: Cintura, 2: Hombro, 3: Muñeca) */
    float i;                        /**< Relación de reducción mecánica entre motor y articulación */
    float currentAngle;             /**< Ángulo actual medido por el encoder magnético [°] */
    float targetAngle;              /**< Ángulo consigna de destino [°] */
    float angleHoming;              /**< Ángulo de referencia de homing [°] */

    float speed;                    /**< Frecuencia instantánea modulada en el timer [Hz] */
    float targetSpeed;              /**< Frecuencia consignada de crucero [Hz] */
    float minSpeed;                 /**< Frecuencia mínima de arranque [Hz] */
    float accel;                    /**< Rampa de aceleración asignada [Hz/s] */

    float Kp;                       /**< Ganancia proporcional sobre el error angular de posición */
    float Kd;                       /**< Ganancia derivativa (modo P2P) */
    float Kg;                       /**< Factor estático de compensación por gravedad */
    float lastAngleForDeriv;        /**< Muestra angular previa para cálculo derivativo [°] */
    float filteredVel;              /**< Velocidad angular estimada con filtro pasa-bajos [°/s] */

    GPIO_TypeDef* DIR_PORT;         /**< Puerto GPIO asignado al pin DIR del DRV8825 */
    uint16_t DIR_PIN;               /**< Pin GPIO asignado al pin DIR del DRV8825 */
    GPIO_PinState dir;              /**< Estado lógico actual del sentido de giro */
    TIM_HandleTypeDef* timer;       /**< Puntero al temporizador hardware asignado */
    uint32_t timerChannel;          /**< Canal del temporizador configurado en PWM */
    MotorState_t state;             /**< Estado operativo actual del actuador */
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor* motors[3];

/**
 * @brief Modifica la frecuencia de conmutación del temporizador PWM actualizando el registro ARR.
 *
 * @param[in,out] htim      Puntero a la estructura TIM_HandleTypeDef.
 * @param[in]     channel   Canal PWM correspondiente (ej. TIM_CHANNEL_1 o TIM_CHANNEL_2).
 * @param[in]     freq_hz   Frecuencia del tren de pulsos solicitada [Hz]. Un valor < 2 Hz detiene el PWM.
 */
void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);

/**
 * @brief Cómputo del par antagonista estático para mitigar el efecto de la gravedad sobre los eslabones.
 *
 * @param[in] m Puntero al descriptor del motor a compensar.
 * @return float Corrección estática calculada en frecuencia equivalente [Hz].
 */
float computeGravityCompensation(Motor *m);

/**
 * @brief Lazo de control articular: prealimentación de velocidad + realimentación proporcional.
 *
 * @param[in,out] m       Puntero al descriptor del actuador.
 * @param[in]     q_ref   Ángulo de referencia instantáneo [°].
 * @param[in]     qd_ref  Velocidad articular feedforward de referencia [°/s].
 * @param[in]     dt      Periodo determinista de muestreo del lazo (0.010 s) [s].
 */
void trajectoryPDControl(Motor *m, float q_ref, float qd_ref, float dt);

/**
 * @brief Conduce la articulación a un ángulo absoluto forzando modo de seguimiento.
 */
void moveToAbsAngle(Motor *m, float angulo_abs);

/**
 * @brief Ejecuta la secuencia individual de búsqueda de origen (homing).
 */
void doHoming(Motor *m);

/**
 * @brief Coordina la secuencia de homing completa para las tres articulaciones.
 */
void doAllHoming(void);

/**
 * @brief Configura e inicia un perfil trapezoidal punto a punto sobre un actuador.
 */
void startP2PMovement(Motor *m, float target_deg, uint32_t speed_hz);

/**
 * @brief Aplica un movimiento punto a punto directo sin validaciones previas de software.
 */
void startDirectP2PMovement(Motor *m, float target_deg, uint32_t speed_hz);

/**
 * @brief Sincroniza cinemáticamente e inicia un movimiento punto a punto coordinado sobre los 3 ejes.
 *
 * @param[in] target_deg Vector de 3 elementos con las consignas angulares deseadas [Q1, Q2, Q3] en [°].
 */
void startSynchronizedP2PMovement(float target_deg[3]);

/**
 * @brief Actualiza la rampa de aceleración/desaceleración trapezoidal en cada ciclo del lazo P2P.
 *
 * @param[in,out] m           Puntero al descriptor del motor.
 * @param[in]     dt_seconds  Tiempo diferencial transcurrido desde la última invocación [s].
 */
void updateP2PRamp(Motor *m, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
