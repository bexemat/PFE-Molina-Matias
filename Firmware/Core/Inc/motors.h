/* Inc/motors.h */
#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

#define ALL_MOTORS_IDLE (motor1.state == IDLE && motor2.state == IDLE && motor3.state == IDLE)

#define SPEED_P2P_DEFAULT_HZ  350.0f
#define MIN_SPEED_HZ          150.0f
#define MAX_V_HZ              1000.0f
#define MAX_A_HZ_S            2000.0f

typedef enum {
    IDLE,
    P2P,
    TRAJ,
    APPROX,
    HOMING,
    MOTOR_ERROR
} MotorState_t;

typedef struct {
    uint8_t id;
    float i;
    float currentAngle;
    float targetAngle;
    float angleHoming;

    float speed;
    float targetSpeed;
    float minSpeed;
    float accel;

    float Kp;
    float Kd;
    float Kg;
    float lastAngleForDeriv;
    float filteredVel;

    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    GPIO_PinState dir;
    TIM_HandleTypeDef* timer;
    uint32_t timerChannel;
    MotorState_t state;
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor* motors[3];

void Stepper_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);
float computeGravityCompensation(Motor *m);
void trajectoryPDControl(Motor *m, float q_ref, float qd_ref, float dt);
void trajectoryControl(Motor *m, float q_ref, float qd_ref);

void moveToAbsAngle(Motor *m, float angulo_abs);
void doHoming(Motor *m);
void doAllHoming(void);
void startP2PMovement(Motor *m, float target_deg, uint32_t speed_hz);
void startDirectP2PMovement(Motor *m, float target_deg, uint32_t speed_hz);
void startSynchronizedP2PMovement(float target_deg[3]);
void updateP2PRamp(Motor *m, float dt_seconds);

#endif /* MOTORS_H */
