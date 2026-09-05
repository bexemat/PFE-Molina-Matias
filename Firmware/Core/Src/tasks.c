/**
 * @file tasks.c
 * @brief Implementación de las tareas concurrentes de tiempo real bajo FreeRTOS y micro-ROS.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Este módulo define los dos hilos principales del sistema operativo:
 *  - **defaultTask (Prioridad Normal):** Inicializa y ejecuta el micro-ROS Client Executor,
 *    administra los publicadores y suscriptores DDS-XRCE y atiende la comunicación serial UART2 por DMA.
 *  - **mControlTask (Alta Prioridad - 100 Hz):** Núcleo de control de movimiento determinista.
 *    Desencola metas cartesianas de mid_PositionQueue, actualiza las rampas de interpolación
 *    quíntica C^2 y ejecuta el lazo de control PD hacia los temporizadores PWM.
 */

#include "tasks.h"
#include "main.h"
#include "motors.h"
#include "kinematics.h"
#include "trajectory_planner.h"
#include "microros_callbacks.h"
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <geometry_msgs/msg/point.h>
#include <extra_interfaces/msg/trama.h>

#define CONTROL_LOOP_DT_SEC (0.010f)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern osMessageQueueId_t mid_PositionQueue;

rcl_publisher_t publisher;
rcl_publisher_t status_publisher;
geometry_msgs__msg__Point tim_msg;
std_msgs__msg__Int8 status_msg;

/** @brief Periodo del temporizador micro-ROS configurado para 40 Hz (25 ms) */
const unsigned int timer_period = RCL_MS_TO_NS(TELEMETRY_TIMER_MS);

/**
 * @brief Tarea de gestión del middleware micro-ROS y ciclo de suscripciones.
 *
 * @param[in] argument Argumento no utilizado requerido por la firma CMSIS-RTOS V2.
 */
void StartDefaultTask(void *argument) {
    (void)argument;

    /* Configuración de la capa de transporte personalizada serial sobre UART2 con DMA */
    rmw_uros_set_custom_transport(true, (void*)&huart2, cubemx_transport_open, cubemx_transport_close,
                                  cubemx_transport_write, cubemx_transport_read);

    /* Asignadores de memoria dinámica ligados al Heap administrado de FreeRTOS */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error: Fallo al asignar allocators FreeRTOS en micro-ROS\r\n");
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    /* Espera activa hasta establecer sincronismo con el micro-ROS Agent en la PC */
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "microros_pubsub_rclc", "", &support));

    /* Inicialización de publicadores ROS 2 */
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "/microROS/angles"
    ));

    RCCHECK(rclc_publisher_init_default(
        &status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/planner/traj_status"
    ));

    /* Temporizador periódico para telemetría a 40 Hz */
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

    /* Declaración e inicialización de suscripciones */
    rcl_subscription_t subs_bool, subs_cmd, subs_p2p, subs_emagnet, subs_estop, subs_req_angles;
    std_msgs__msg__Bool homing_msg, emagnet_msg, estop_msg, req_angles_msg;
    extra_interfaces__msg__Trama cmd_msg;
    geometry_msgs__msg__Point p2p_msg;

    std_msgs__msg__Bool__init(&homing_msg);
    extra_interfaces__msg__Trama__init(&cmd_msg);
    geometry_msgs__msg__Point__init(&p2p_msg);
    std_msgs__msg__Bool__init(&emagnet_msg);
    std_msgs__msg__Bool__init(&estop_msg);
    std_msgs__msg__Bool__init(&req_angles_msg);
    geometry_msgs__msg__Point__init(&tim_msg);
    std_msgs__msg__Int8__init(&status_msg);

    RCCHECK(rclc_subscription_init_default(&subs_bool, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/homing"));
    RCCHECK(rclc_subscription_init_default(&subs_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(extra_interfaces, msg, Trama), "/microROS/cmd"));
    RCCHECK(rclc_subscription_init_default(&subs_p2p, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/microROS/p2p_cmd"));
    RCCHECK(rclc_subscription_init_default(&subs_emagnet, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/electroiman"));
    RCCHECK(rclc_subscription_init_default(&subs_estop, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/emergency_stop"));
    RCCHECK(rclc_subscription_init_default(&subs_req_angles, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/microROS/request_current_angles"));

    /* Instanciación del executor micro-ROS para 7 entidades registradas (6 suscriptores + 1 timer) */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &subs_bool, &homing_msg, &homing_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_cmd, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_p2p, &p2p_msg, &p2p_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_emagnet, &emagnet_msg, &electromagnet_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_estop, &estop_msg, &estop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_req_angles, &req_angles_msg, &request_angles_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    /* Bucle principal del hilo de comunicación */
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Tarea determinista de alta prioridad para cinemática, planificación y control de actuadores (100 Hz).
 *
 * @param[in] argument Argumento no utilizado requerido por la firma CMSIS-RTOS V2.
 */
void StartMotorControlTask(void *argument) {
    (void)argument;
    osStatus_t xStatus;
    CARTESIAN_CMD_t cart_cmd;
    float q_cmd[3] = {0.0f};
    float qd_cmd[3] = {0.0f};

    TrajectoryPlanner_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);

    for (;;) {
        /* 1. Actualización de perfiles trapezoidales en ejes en modo P2P */
        for (int i = 0; i < 3; i++) {
            if (motors[i]->state == P2P) {
                updateP2PRamp(motors[i], CONTROL_LOOP_DT_SEC);
            }
        }

        /* 2. Desencolado no bloqueante de nueva consigna cartesiana única */
        xStatus = osMessageQueueGet(mid_PositionQueue, &cart_cmd, NULL, 0U);
        if (xStatus == osOK) {
            const float target_xyz[3] = {cart_cmd.x, cart_cmd.y, cart_cmd.z};
            const float current_q[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};

            if (TrajectoryPlanner_StartCtraj(target_xyz, cart_cmd.duration, current_q)) {
                for (int i = 0; i < 3; i++) {
                    motors[i]->state = TRAJ;
                }
                status_msg.data = 1; /* Estado 1: En ejecución de trayectoria */
                rcl_publish(&status_publisher, &status_msg, NULL);
            } else {
                status_msg.data = -1; /* Estado -1: Error cinemático / Punto inalcanzable */
                rcl_publish(&status_publisher, &status_msg, NULL);
            }
        }

        /* 3. Lazo determinista de seguimiento quíntico y control PD a 100 Hz */
        if (g_traj_planner.is_active) {
            const float current_q[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
            const bool in_progress = TrajectoryPlanner_Update(CONTROL_LOOP_DT_SEC, q_cmd, qd_cmd, current_q);

            for (int i = 0; i < 3; i++) {
                motors[i]->targetAngle = q_cmd[i];
                trajectoryPDControl(motors[i], q_cmd[i], qd_cmd[i], CONTROL_LOOP_DT_SEC);
            }

            if (!in_progress) {
                for (int i = 0; i < 3; i++) {
                    motors[i]->state = IDLE;
                    motors[i]->speed = 0.0f;
                    HAL_TIM_PWM_Stop(motors[i]->timer, motors[i]->timerChannel);
                }
                status_msg.data = 2; /* Estado 2: Trayectoria completada y asentamiento finalizado */
                rcl_publish(&status_publisher, &status_msg, NULL);
            }
        }

        /* Ejecución determinista estricta cada 10 ms */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Inicializa e instancía los hilos de ejecución de la aplicación en el scheduler.
 */
void MX_Tasks_Init(void) {
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    mControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &mControlTask_attributes);
}

#include <unistd.h>

/**
 * @brief Implementación de usleep para compatibilidad POSIX sobre RTOS.
 *
 * @param[in] usec Tiempo de espera en microsegundos [us].
 * @return int Retorna 0 tras completarse el retardo.
 */
int usleep(useconds_t usec) {
    const uint32_t ms = usec / 1000U;
    if (ms == 0U) {
        const uint32_t loops = (uint32_t)((usec * (SystemCoreClock / 1000000UL)) / 4U);
        for (volatile uint32_t i = 0; i < loops; i++) {
            __NOP();
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    return 0;
}
