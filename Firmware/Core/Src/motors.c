/* Src/motors.c */
#include "motors.h"
#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define STEPS_PER_REV   (200 * 8)
#define DEG_TO_STEPS    ((float)STEPS_PER_REV / 360.0f)

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

Motor* motors[] = { &motor1, &motor2, &motor3 };

void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz) {
    if (freq_hz < 2) {
        HAL_TIM_PWM_Stop(htim, channel);
        return;
    }

    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
    uint32_t prescaler = htim->Instance->PSC + 1;
    uint32_t div = prescaler * freq_hz;

    if (div == 0 || timer_clk < div) return;

    uint32_t arr = (timer_clk / div) - 1;

    if (htim->Instance == TIM3 || htim->Instance == TIM13) {
        if (arr > 65535) arr = 65535;
    }

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);
}

float computeGravityCompensation(Motor *m) {
    if (m->Kg == 0.0f) return 0.0f;

    float rad2 = motor2.currentAngle * DEG_TO_RAD;
    float rad3 = motor3.currentAngle * DEG_TO_RAD;

    switch (m->id) {
        case 1: return 0.0f;
        case 2: return m->Kg * cosf(rad2) + (m->Kg * 0.5f) * cosf(rad2 + rad3);
        case 3: return m->Kg * cosf(rad2 + rad3);
        default: return 0.0f;
    }
}

void trajectoryPDControl(Motor *m, float q_ref, float qd_ref, float dt) {
    if (dt <= 0.0001f) return;

    float q_err = q_ref - m->currentAngle;
    float v_cmd = 0.0f;

    if (m->state == TRAJ) {
        if (fabsf(q_err) <= 0.05f) {
            q_err = 0.0f;
        }

        // Ganancia de seguimiento y convergencia final
        float Kp_traj = 3.50f;
        v_cmd = qd_ref + (Kp_traj * q_err);
        m->lastAngleForDeriv = m->currentAngle;

    } else if (m->state == P2P) {
        if (fabsf(q_err) <= ANGLE_TOLERANCE) {
            m->speed = 0.0f;
            HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
            m->state = IDLE;
            return;
        }

        float raw_vel = (m->currentAngle - m->lastAngleForDeriv) / dt;
        m->filteredVel = (0.20f * raw_vel) + (0.80f * m->filteredVel);
        v_cmd = (m->Kp * q_err) - (m->Kd * m->filteredVel);
        m->lastAngleForDeriv = m->currentAngle;

    } else {
        m->speed = 0.0f;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    float v_hz = fabsf(v_cmd) * DEG_TO_STEPS * m->i;

    if (v_hz < 1.0f) {
        v_hz = 0.0f;
        m->speed = 0.0f;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    if (v_hz > MAX_V_HZ) v_hz = MAX_V_HZ;

    m->dir = (v_cmd >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    m->speed = v_hz;

    HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);
    Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
    HAL_TIM_PWM_Start(m->timer, m->timerChannel);
}

void trajectoryControl(Motor *m, float q_ref, float qd_ref) {
    trajectoryPDControl(m, q_ref, qd_ref, 0.010f);
}

void startSynchronizedP2PMovement(float target_deg[3]) {
    float delta_s[3] = {0.0f};
    float t_req[3] = {0.0f};
    float t_max = 0.0f;

    for (int i = 0; i < 3; i++) {
        Motor* m = motors[i];
        m->targetAngle = target_deg[i];
        float delta_deg = fabsf(target_deg[i] - m->currentAngle);

        if (delta_deg <= ANGLE_TOLERANCE) {
            delta_s[i] = 0.0f;
            t_req[i] = 0.0f;
            continue;
        }

        delta_s[i] = delta_deg * DEG_TO_STEPS * m->i;
        float s_crit = (MAX_V_HZ * MAX_V_HZ) / MAX_A_HZ_S;

        if (delta_s[i] >= s_crit) {
            t_req[i] = (MAX_V_HZ / MAX_A_HZ_S) + (delta_s[i] / MAX_V_HZ);
        } else {
            t_req[i] = 2.0f * sqrtf(delta_s[i] / MAX_A_HZ_S);
        }

        if (t_req[i] > t_max) {
            t_max = t_req[i];
        }
    }

    if (t_max <= 0.01f) return;

    for (int i = 0; i < 3; i++) {
        Motor* m = motors[i];

        if (delta_s[i] == 0.0f) {
            m->state = IDLE;
            continue;
        }

        float err = target_deg[i] - m->currentAngle;
        m->dir = (err > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, m->dir);

        float t_accel = 0.20f * t_max;
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

void updateP2PRamp(Motor *m, float dt_seconds) {
    if (m->state != P2P) return;

    float dist_remaining = fabsf(m->targetAngle - m->currentAngle);

    if (dist_remaining <= ANGLE_TOLERANCE) {
        m->speed = 0.0f;
        m->state = IDLE;
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
        return;
    }

    if (m->speed < m->targetSpeed) {
        m->speed += m->accel * dt_seconds;
        if (m->speed > m->targetSpeed) {
            m->speed = m->targetSpeed;
        }
    }

    float decel_zone_deg = 2.5f;
    if (dist_remaining < decel_zone_deg) {
        float factor = dist_remaining / decel_zone_deg;
        float decel_speed = m->targetSpeed * factor;
        if (decel_speed < m->minSpeed) {
            decel_speed = m->minSpeed;
        }
        m->speed = decel_speed;
    }

    Stepper_SetSpeed(m->timer, m->timerChannel, (uint32_t)m->speed);
}

void doAllHoming(void) {
    float homing_targets[3] = {
        motor1.angleHoming,
        motor2.angleHoming,
        motor3.angleHoming
    };
    startSynchronizedP2PMovement(homing_targets);
}

void doHoming(Motor *m) {
    float targets[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
    targets[m->id - 1] = m->angleHoming;
    startSynchronizedP2PMovement(targets);
}

void startP2PMovement(Motor *m, float target_deg, uint32_t speed_hz) {
    float targets[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
    targets[m->id - 1] = target_deg;
    startSynchronizedP2PMovement(targets);
}

void moveToAbsAngle(Motor *m, float angulo_abs) {
    startP2PMovement(m, angulo_abs, (uint32_t)SPEED_P2P_DEFAULT_HZ);
}

void startDirectP2PMovement(Motor *m, float target_deg, uint32_t speed_hz) {
    m->targetAngle = target_deg;
    float err = target_deg - m->currentAngle;

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
