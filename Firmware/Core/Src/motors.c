/**
 * @file motors.c
 * @brief Accionamiento de bajo nivel de motores paso a paso y lazo de control PD.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Este módulo gestiona los controladores DRV8825 mediante modulación PWM por hardware
 * en temporizadores STM32 (TIM2, TIM3, TIM13) y control digital de dirección por GPIO.
 * Implementa lazos de control de velocidad y posición para movimientos punto a punto (P2P)
 * sincronizados y seguimiento de trayectorias con compensación estática de gravedad.
 */

#include "motors.h"
#include "main.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F (3.14159265358979323846f)
#endif

#define DEG_TO_RAD_M    (M_PI_F / 180.0f)
#define STEPS_PER_REV   (200 * 8)                 /**< 200 pasos/rev * 8 micropasos = 1600 pulsos/rev */
#define DEG_TO_STEPS    ((float)STEPS_PER_REV / 360.0f) /**< Factor de escala angular a pulsos [pulsos/°] */

/* =========================================================================
 * INSTANCIAS GLOBALES DE DESCRIPTORES DE MOTOR
 * ========================================================================= */

/** @brief Descriptor del Motor 1: Cintura (Eje azimutal) */
Motor motor1 = {
    .id = 1,
    .i = 108.0f / 19.0f,
    .angleHoming = 0.0f,
    .DIR_PORT = DIR1_GPIO,
    .DIR_PIN = DIR1_PIN,
    .timer = &htim13,
    .timerChannel = TIM_CHANNEL_1,
    .state = IDLE,
    .speed = 0.0f,
    .targetSpeed = MAX_V_HZ,
    .minSpeed = MIN_SPEED_HZ,
    .accel = MAX_A_HZ_S,
    .Kp = 4.5f,
    .Kd = 0.10f,
    .Kg = 0.0f,
    .lastAngleForDeriv = 0.0f,
    .filteredVel = 0.0f
};

/** @brief Descriptor del Motor 2: Hombro (Eje principal de elevación) */
Motor motor2 = {
    .id = 2,
    .i = 32.0f / 12.0f,
    .angleHoming = 90.0f,
    .DIR_PORT = DIR2_GPIO,
    .DIR_PIN = DIR2_PIN,
    .timer = &htim2,
    .timerChannel = TIM_CHANNEL_2,
    .state = IDLE,
    .speed = 0.0f,
    .targetSpeed = MAX_V_HZ,
    .minSpeed = MIN_SPEED_HZ,
    .accel = MAX_A_HZ_S,
    .Kp = 5.0f,
    .Kd = 0.18f,
    .Kg = 0.0f,
    .lastAngleForDeriv = 90.0f,
    .filteredVel = 0.0f
};

/** @brief Descriptor del Motor 3: Muñeca (Eje de antebrazo / paralelogramo) */
Motor motor3 = {
    .id = 3,
    .i = 32.0f / 12.0f,
    .angleHoming = 0.0f,
    .DIR_PORT = DIR3_GPIO,
    .DIR_PIN = DIR3_PIN,
    .timer = &htim3,
    .timerChannel = TIM_CHANNEL_2,
    .state = IDLE,
    .speed = 0.0f,
    .targetSpeed = MAX_V_HZ,
    .minSpeed = MIN_SPEED_HZ,
    .accel = MAX_A_HZ_S,
    .Kp = 4.5f,
    .Kd = 0.12f,
    .Kg = 0.0f,
    .lastAngleForDeriv = 0.0f,
    .filteredVel = 0.0f
};

/** @brief Arreglo de punteros para acceso indexado a los actuadores */
Motor* motors[] = { &motor1, &motor2, &motor3 };

/**
 * @brief Configura directamente el registro Auto-Reload (ARR) del temporizador para ajustar la frecuencia PWM.
 *
 * @details Calcula el periodo ARR asegurando un ciclo de trabajo del 50% (CCR = ARR / 2).
 * Aplica saturación a 16 bits para los temporizadores TIM3 y TIM13.
 *
 * @param[in,out] htim      Puntero a la estructura de control del periférico TIM_HandleTypeDef.
 * @param[in]     channel   Canal PWM a modular.
 * @param[in]     freq_hz   Frecuencia solicitada en hercios [Hz]. Frecuencias menores a 2 Hz apagan el PWM.
 */
void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz) {
    if (freq_hz < 2U) {
        HAL_TIM_PWM_Stop(htim, channel);
        return;
    }

    /* El reloj de los timers APB1 opera al doble de PCLK1 cuando el prescaler APB1 es distinto de 1 */
    const uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2U;
    const uint32_t prescaler = htim->Instance->PSC + 1U;
    const uint32_t div = prescaler * freq_hz;

    if ((div == 0U) || (timer_clk < div)) {
        return;
    }

    uint32_t arr = (timer_clk / div) - 1U;

    /* Saturación estricta para temporizadores de 16 bits (TIM3 y TIM13) */
    if ((htim->Instance == TIM3) || (htim->Instance == TIM13)) {
        if (arr > 65535U) {
            arr = 65535U;
        }
    }

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2U);
}

/**
 * @brief Cómputo del par antagonista estático para mitigar el efecto de la gravedad sobre los eslabones.
 *
 * @param[in] m Puntero al descriptor del motor a compensar.
 * @return float Corrección estática calculada en frecuencia equivalente [Hz].
 */
float computeGravityCompensation(Motor *m) {
    if (m->Kg == 0.0f) {
        return 0.0f;
    }

    const float rad2 = motor2.currentAngle * DEG_TO_RAD_M;
    const float rad3 = motor3.currentAngle * DEG_TO_RAD_M;

    switch (m->id) {
        case 1:
            return 0.0f; /* El eje de cintura es perpendicular a la fuerza de gravedad */
        case 2:
            return m->Kg * cosf(rad2) + (m->Kg * 0.5f) * cosf(rad2 + rad3);
        case 3:
            return m->Kg * cosf(rad2 + rad3);
        default:
            return 0.0f;
    }
}

/**
 * @brief Lazo de control PD con término feedforward de velocidad para seguimiento de trayectorias.
 *
 * @details Calcula la velocidad angular requerida:
 * \f[
 *   v_{cmd} = \dot{q}_{ref} + K_{p\_traj} (q_{ref} - q_{real})
 * \f]
 * Transforma el comando a pulsos por segundo [Hz] considerando la reducción mecánica y el micropaseado,
 * actualiza el pin de dirección (DIR) y modula el temporizador PWM.
 *
 * @param[in,out] m       Puntero al descriptor del motor.
 * @param[in]     q_ref   Ángulo articular de referencia [°].
 * @param[in]     qd_ref  Velocidad articular feedforward de referencia [°/s].
 * @param[in]     dt      Periodo de muestreo del lazo (0.010 s) [s].
 */
void trajectoryPDControl(Motor *m, float q_ref, float qd_ref, float dt) {
    if (dt <= 0.0001f) {
        return;
    }

    float q_err = q_ref - m->currentAngle;
    float v_cmd = 0.0f;

    if (m->state == TRAJ) {
        /* Zona muerta mínima para evitar sobreoscilaciones de micro-vibración */
        if (fabsf(q_err) <= 0.05f) {
            q_err = 0.0f;
        }

        /* Ganancia de seguimiento cinemático */
        const float Kp_traj = 3.50f;
        v_cmd = qd_ref + (Kp_traj * q_err);
        m->lastAngleForDeriv = m->currentAngle;

    } else if (m->state == P2P) {
        /* Manejo de modo punto a punto con filtro derivativo sobre la velocidad real */
        if (fabsf(q_err) <= ANGLE_TOLERANCE) {
            m->speed = 0.0f;
            HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
            m->state = IDLE;
            return;
        }

        const float raw_vel = (m->currentAngle - m->lastAngleForDeriv) / dt;
        m->filteredVel = (0.20f * raw_vel) + (0.80f * m->filteredVel);
        v_cmd = (m->Kp * q_err) - (m->Kd * m->filteredVel);
        m->lastAngleForDeriv = m->currentAngle;

    } else {
        m->speed = 0.0f;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    /* Conversión de velocidad angular [°/s] a frecuencia de pulsos [Hz] */
    float v_hz = fabsf(v_cmd) * DEG_TO_STEPS * m->i;

    /* Umbral de parada para velocidades mínimas */
    if (v_hz < 1.0f) {
        v_hz = 0.0f;
        m->speed = 0.0f;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    /* Saturación por límite dinámico superior */
    if (v_hz > MAX_V_HZ) {
        v_hz = MAX_V_HZ;
    }

    /* Asignación física del sentido de giro por GPIO */
    m->dir = (v_cmd >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    m->speed = v_hz;

    HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);
    Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
    HAL_TIM_PWM_Start(m->timer, m->timerChannel);
}

/**
 * @brief Adaptador para control de trayectoria con periodo nominal preestablecido de 10 ms.
 */
void trajectoryControl(Motor *m, float q_ref, float qd_ref) {
    trajectoryPDControl(m, q_ref, qd_ref, 0.010f);
}

/**
 * @brief Sincroniza y planifica el movimiento coordinado punto a punto sobre las tres articulaciones.
 *
 * @details Determina cuál de los ejes demanda el mayor tiempo de desplazamiento bajo perfiles trapezoidales
 * y escala proporcionalmente la velocidad y aceleración de los otros dos ejes para asegurar que todos
 * los actuadores inicien y concluyan su recorrido exactamente en el mismo instante.
 *
 * @param[in] target_deg Vector de 3 elementos con las consignas articulares [Q1, Q2, Q3] en [°].
 */
void startSynchronizedP2PMovement(float target_deg[3]) {
    float delta_s[3] = {0.0f};
    float t_req[3] = {0.0f};
    float t_max = 0.0f;

    /* Paso 1: Determinar el tiempo de viaje crítico para cada eje individual */
    for (int i = 0; i < 3; i++) {
        Motor* m = motors[i];
        m->targetAngle = target_deg[i];
        const float delta_deg = fabsf(target_deg[i] - m->currentAngle);

        if (delta_deg <= ANGLE_TOLERANCE) {
            delta_s[i] = 0.0f;
            t_req[i] = 0.0f;
            continue;
        }

        delta_s[i] = delta_deg * DEG_TO_STEPS * m->i;
        const float s_crit = (MAX_V_HZ * MAX_V_HZ) / MAX_A_HZ_S;

        if (delta_s[i] >= s_crit) {
            /* Perfil trapezoidal completo (alcanza velocidad de crucero) */
            t_req[i] = (MAX_V_HZ / MAX_A_HZ_S) + (delta_s[i] / MAX_V_HZ);
        } else {
            /* Perfil triangular (no alcanza la velocidad máxima) */
            t_req[i] = 2.0f * sqrtf(delta_s[i] / MAX_A_HZ_S);
        }

        if (t_req[i] > t_max) {
            t_max = t_req[i];
        }
    }

    if (t_max <= 0.01f) {
        return;
    }

    /* Paso 2: Escalar cinemáticamente los ejes secundarios al tiempo del eje más lento */
    for (int i = 0; i < 3; i++) {
        Motor* m = motors[i];

        if (delta_s[i] == 0.0f) {
            m->state = IDLE;
            continue;
        }

        const float err = target_deg[i] - m->currentAngle;
        m->dir = (err > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);

        const float t_accel = 0.20f * t_max;
        float v_scaled = delta_s[i] / (0.80f * t_max);
        float a_scaled = v_scaled / t_accel;

        if (a_scaled > MAX_A_HZ_S) {
            v_scaled = (2.0f * delta_s[i]) / t_max;
            a_scaled = (4.0f * delta_s[i]) / (t_max * t_max);
        }

        if (v_scaled < 280.0f) {
            v_scaled = 280.0f;
            a_scaled = MAX_A_HZ_S;
        } else if (v_scaled > MAX_V_HZ) {
            v_scaled = MAX_V_HZ;
        }

        m->minSpeed = MIN_SPEED_HZ;
        m->targetSpeed = v_scaled;
        m->speed = m->minSpeed;
        m->accel = a_scaled;
        m->state = P2P;
        m->lastAngleForDeriv = m->currentAngle;

        Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
        HAL_TIM_PWM_Start(m->timer, m->timerChannel);
        HAL_TIM_Base_Start_IT(m->timer);
    }
}

/**
 * @brief Actualiza la rampa trapezoidal de aceleración/deceleración en cada iteración del lazo P2P.
 *
 * @param[in,out] m           Puntero al descriptor del motor.
 * @param[in]     dt_seconds  Tiempo diferencial transcurrido desde la última llamada [s].
 */
void updateP2PRamp(Motor *m, float dt_seconds) {
    if (m->state != P2P) {
        return;
    }

    const float dist_remaining = fabsf(m->targetAngle - m->currentAngle);

    if (dist_remaining <= ANGLE_TOLERANCE) {
        m->speed = 0.0f;
        m->state = IDLE;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    /* Rampa de aceleración ascendente */
    if (m->speed < m->targetSpeed) {
        m->speed += m->accel * dt_seconds;
        if (m->speed > m->targetSpeed) {
            m->speed = m->targetSpeed;
        }
    }

    /* Zona de deceleración lineal terminal */
    const float decel_zone_deg = 2.5f;
    if (dist_remaining < decel_zone_deg) {
        const float factor = dist_remaining / decel_zone_deg;
        float decel_speed = m->targetSpeed * factor;
        if (decel_speed < m->minSpeed) {
            decel_speed = m->minSpeed;
        }
        m->speed = decel_speed;
    }

    Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
}

/**
 * @brief Inicia la secuencia de calibración al origen (homing) para todas las articulaciones.
 */
void doAllHoming(void) {
    float homing_targets[3] = {
        motor1.angleHoming,
        motor2.angleHoming,
        motor3.angleHoming
    };
    startSynchronizedP2PMovement(homing_targets);
}

/**
 * @brief Inicia la secuencia de referenciamiento individual sobre un motor.
 */
void doHoming(Motor *m) {
    float targets[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
    targets[m->id - 1] = m->angleHoming;
    startSynchronizedP2PMovement(targets);
}

/**
 * @brief Desplaza una articulación a una meta angular manteniendo inmóviles las demás.
 */
void startP2PMovement(Motor *m, float target_deg, uint32_t speed_hz) {
    (void)speed_hz;
    float targets[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
    targets[m->id - 1] = target_deg;
    startSynchronizedP2PMovement(targets);
}

/**
 * @brief Mueve el motor a un ángulo absoluto usando la velocidad por defecto.
 */
void moveToAbsAngle(Motor *m, float angulo_abs) {
    startP2PMovement(m, angulo_abs, (uint32_t)SPEED_P2P_DEFAULT_HZ);
}

/**
 * @brief Comanda un movimiento punto a punto directo sin sincronización multi-eje.
 */
void startDirectP2PMovement(Motor *m, float target_deg, uint32_t speed_hz) {
    m->targetAngle = target_deg;
    const float err = target_deg - m->currentAngle;

    if (fabsf(err) <= ANGLE_TOLERANCE) {
        m->state = IDLE;
        return;
    }

    m->dir = (err > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);

    m->state = P2P;
    m->speed = (float)speed_hz;
    m->lastAngleForDeriv = m->currentAngle;

    Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
    HAL_TIM_PWM_Start(m->timer, m->timerChannel);
    HAL_TIM_Base_Start_IT(m->timer);
}
