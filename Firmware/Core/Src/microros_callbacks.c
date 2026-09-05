/**
 * @file microros_callbacks.c
 * @brief Callbacks de procesamiento asíncrono para la capa de comunicación micro-ROS.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Este módulo gestiona los callbacks invocados por el executor de micro-ROS:
 *  - Encolado seguro de consignas cartesianas (/microROS/cmd).
 *  - Comandos directos punto a punto en espacio articular (/microROS/p2p_cmd).
 *  - Coordinación de la calibración al origen (/microROS/homing).
 *  - Accionamiento del electroimán y parada de emergencia por hardware (/microROS/emergency_stop).
 *  - Publicación periódica determinista de la pose angular a 40 Hz (/microROS/angles).
 */

#include "microros_callbacks.h"
#include "electromagnet.h"
#include "motors.h"
#include "trajectory_planner.h"
#include "cmsis_os.h"
#include <stdio.h>

extern osMessageQueueId_t mid_PositionQueue;
extern rcl_publisher_t publisher;
extern geometry_msgs__msg__Point tim_msg;

/**
 * @brief Callback de recepción de metas cartesianas estructuradas (/microROS/cmd).
 *
 * @details Empaqueta la trama recibida en una estructura CARTESIAN_CMD_t y la deposita
 * de forma no bloqueante en mid_PositionQueue para su consumo determinista por mControlTask a 100 Hz.
 *
 * @param[in] msgin Puntero opaco al mensaje recibido (extra_interfaces__msg__Trama).
 */
void cmd_callback(const void * msgin) {
    const extra_interfaces__msg__Trama * msg = (const extra_interfaces__msg__Trama *)msgin;

    const CARTESIAN_CMD_t cmd = {
        .x = (float)msg->q[0],
        .y = (float)msg->q[1],
        .z = (float)msg->q[2],
        .duration = (float)msg->t_total
    };

    osMessageQueuePut(mid_PositionQueue, &cmd, 0U, 0U);
}

/**
 * @brief Callback para comandos directos en espacio articular (/microROS/p2p_cmd).
 *
 * @param[in] msgin Puntero opaco al mensaje recibido (geometry_msgs__msg__Point).
 */
void p2p_cmd_callback(const void * msgin) {
    const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
    float targets[3] = {(float)msg->x, (float)msg->y, (float)msg->z};
    startSynchronizedP2PMovement(targets);
}

/**
 * @brief Callback para inicio de la rutina de homing articular (/microROS/homing).
 *
 * @param[in] msgin Puntero opaco al mensaje recibido (std_msgs__msg__Bool).
 */
void homing_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    if (msg->data) {
        doAllHoming();
    }
}

/**
 * @brief Callback para conmutación del efector final magnético (/microROS/electroiman).
 *
 * @param[in] msgin Puntero opaco al mensaje recibido (std_msgs__msg__Bool).
 */
void electromagnet_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    electromagnetOn(msg->data);
}

/**
 * @brief Callback prioritario para la parada de emergencia del sistema (/microROS/emergency_stop).
 *
 * @details Cancela de forma inmediata la trayectoria cartesiana activa, interrumpe
 * los trenes de pulsos PWM por hardware en los tres temporizadores y conmuta el estado de
 * los actuadores a MOTOR_ERROR para evitar daños mecánicos o térmicos.
 *
 * @param[in] msgin Puntero opaco al mensaje recibido (std_msgs__msg__Bool).
 */
void estop_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    const bool estop_engaged = msg->data;

    if (estop_engaged) {
        TrajectoryPlanner_Abort();
        for (int i = 0; i < 3; i++) {
            motors[i]->state = MOTOR_ERROR;
            HAL_TIM_PWM_Stop(motors[i]->timer, motors[i]->timerChannel);
            HAL_TIM_Base_Stop_IT(motors[i]->timer);
        }
    } else {
        for (int i = 0; i < 3; i++) {
            motors[i]->state = IDLE;
        }
    }
}

/**
 * @brief Callback para atender solicitudes discretas de la pose articular actual.
 */
void request_angles_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        tim_msg.x = (double)motor1.currentAngle;
        tim_msg.y = (double)motor2.currentAngle;
        tim_msg.z = (double)motor3.currentAngle;
        rcl_publish(&publisher, &tim_msg, NULL);
    }
}

/**
 * @brief Callback periódico del temporizador micro-ROS (40 Hz): publica la telemetría angular.
 *
 * @param[in] timer           Puntero a la estructura del timer rcl_timer_t.
 * @param[in] last_call_time  Marca de tiempo de la invocación previa en nanosegundos.
 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    tim_msg.x = (double)motor1.currentAngle;
    tim_msg.y = (double)motor2.currentAngle;
    tim_msg.z = (double)motor3.currentAngle;

    if (timer != NULL) {
        rcl_publish(&publisher, &tim_msg, NULL);
    }
}
