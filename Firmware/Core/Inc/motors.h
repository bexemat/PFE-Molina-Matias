/**
 * @file motors.h
 * @brief Controlador de actuadores paso a paso con drivers DRV8825 y lazo cerrado AS5600.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Modula los trenes de pulso por temporizadores hardware (TIM2, TIM3, TIM13),
 * gestiona señales de sentido de giro por GPIO y ejecuta perfiles trapezoidales y de
 * asentamiento PD con compensación estática de gravedad.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALL_MOTORS_IDLE ((motor1.state == IDLE) && (motor2.state == IDLE) && (motor3.state == IDLE))

/* =========================================================================
 * PARÁMETROS CINEMÁTICOS DE ACCIONAMIENTO (1/8 Micropaso)
 * ========================================================================= */
#define SPEED_P2P_DEFAULT_HZ  (350.0f)   /**< Velocidad angular por defecto para traslados punto a punto [Hz] */
#define MIN_SPEED_HZ          (150.0f)   /**< Velocidad base de arranque para evitar pérdida de paso [Hz] */
#define MAX_V_HZ              (1000.0f)  /**< Límite superior absoluto de velocidad por temporizador [Hz] */
#define MAX_A_HZ_S            (2000.0f)  /**< Rampa máxima de aceleración admisible [Hz/s] */

/**
 * @brief Estados operativos de cada actuador.
 */
typedef enum {
    IDLE = 0,       /**< Actuador en reposo con excitación estática */
    P2P,            /**< Movimiento articular punto a punto con rampa trapezoidal */
    TRAJ,           /**< Seguimiento de trayectoria polinómica continua */
    APPROX,         /**< Lazo de aproximación final de alta precisión */
    HOMING,         /**< Búsqueda activa de posición de referencia */
    MOTOR_ERROR     /**< Condición de error o parada de emergencia */
} MotorState_t;

/**
 * @brief Descriptor de hardware y parámetros de control de cada eje del manipulador.
 */
typedef struct {
    uint8_t id;                     /**< Identificador del eje (1: Cintura, 2: Hombro, 3: Muñeca) */
    float i;                        /**< Relación de reducción mecánica entre motor y articulación */
    float currentAngle;             /**< Ángulo actual leído por el sensor magnético [°] */
    float targetAngle;              /**< Ángulo consigna de destino [°] */
    float angleHoming;              /**< Ángulo de referencia de homing [°] */

    float speed;                    /**< Frecuencia instantánea de pulsos aplicada [Hz] */
    float targetSpeed;              /**< Frecuencia de régimen consignada [Hz] */
    float minSpeed;                 /**< Frecuencia mínima de arranque [Hz] */
    float accel;                    /**< Rampa de aceleración configurada [Hz/s] */

    float Kp;                       /**< Ganancia proporcional para corrección angular */
    float Kd;                       /**< Ganancia derivativa */
    float Kg;                       /**< Factor estático de compensación por gravedad */
    float lastAngleForDeriv;        /**< Muestra previa de ángulo para cálculo derivativo [°] */
    float filteredVel;              /**< Velocidad angular calculada y filtrada [°/s] */

    GPIO_TypeDef* DIR_PORT;         /**< Puerto GPIO para pin DIR */
    uint16_t DIR_PIN;               /**< Pin GPIO para pin DIR */
    GPIO_PinState dir;              /**< Estado lógico actual del sentido de giro */
    TIM_HandleTypeDef* timer;       /**< Manejador del temporizador hardware */
    uint32_t timerChannel;          /**< Canal del temporizador configurado en PWM */
    MotorState_t state;             /**< Estado operativo en la máquina de estados */
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor* motors[3];

/**
 * @brief Modifica la frecuencia de conmutación del temporizador PWM por hardware sin jitter.
 *
 * @param[in,out] htim      Puntero a la estructura TIM_HandleTypeDef.
 * @param[in]     channel   Canal PWM correspondiente (ej. TIM_CHANNEL_1).
 * @param[in]     freq_hz   Frecuencia del tren de pulsos en hercios [Hz]. Frecuencia 0 desactiva la salida.
 */
void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);

/**
 * @brief Calcula el término de compensación por gravedad según la pose articular actual.
 *
 * @param[in] m Puntero al descriptor del motor.
 * @return float Componente de corrección en frecuencia [Hz].
 */
float computeGravityCompensation(Motor *m);

/**
 * @brief Lazo de control PD de seguimiento de trayectoria con compensación feedforward.
 *
 * @param[in,out] m       Puntero al descriptor del motor.
 * @param[in]     q_ref   Ángulo de referencia instantáneo [°].
 * @param[in]     qd_ref  Velocidad angular de referencia deseada [°/s].
 * @param[in]     dt      Periodo de muestreo [s].
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
 * @brief Sincroniza e inicia un movimiento punto a punto coordinado sobre los 3 ejes.
 *
 * @param[in] target_deg Vector de 3 elementos con las consignas angulares deseadas [°].
 */
void startSynchronizedP2PMovement(float target_deg[3]);

/**
 * @brief Actualiza la rampa de aceleración/desaceleración trapezoidal en cada ciclo.
 *
 * @param[in,out] m           Puntero al descriptor del motor.
 * @param[in]     dt_seconds  Tiempo diferencial transcurrido [s].
 */
void updateP2PRamp(Motor *m, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
