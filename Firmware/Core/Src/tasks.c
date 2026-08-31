/* Src/tasks.c */
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

#define CONTROL_LOOP_PERIOD_MS  10
#define CONTROL_LOOP_DT_SEC     0.010f

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern osMessageQueueId_t mid_PositionQueue;

rcl_publisher_t publisher;
rcl_publisher_t status_publisher;
geometry_msgs__msg__Point tim_msg;
std_msgs__msg__Int8 status_msg;

const unsigned int timer_period = RCL_MS_TO_NS(25); // 40 Hz de telemetria

void StartDefaultTask(void *argument)
{
    rmw_uros_set_custom_transport(true, (void*) &huart2, cubemx_transport_open, cubemx_transport_close,
            cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error en allocators\n");
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "microros_pubsub_rclc", "", &support));

    // Publishers
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

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

    // Subscriptions
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

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &subs_bool, &homing_msg, &homing_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_cmd, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_p2p, &p2p_msg, &p2p_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_emagnet, &emagnet_msg, &electromagnet_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_estop, &estop_msg, &estop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subs_req_angles, &req_angles_msg, &request_angles_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void StartMotorControlTask(void *argument) {
    osStatus_t xStatus;
    CARTESIAN_CMD_t cart_cmd;
    float q_cmd[3] = {0.0f};
    float qd_cmd[3] = {0.0f};

    TrajectoryPlanner_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);

    for (;;) {
        // 1. Rampas P2P para movimientos articulares directos
        for (int i = 0; i < 3; i++) {
            if (motors[i]->state == P2P) {
                updateP2PRamp(motors[i], CONTROL_LOOP_DT_SEC);
            }
        }

        // 2. Recepcion de nueva consigna cartesiana unica (X, Y, Z + T)
        xStatus = osMessageQueueGet(mid_PositionQueue, &cart_cmd, NULL, 0);
        if (xStatus == osOK) {
            float target_xyz[3] = {cart_cmd.x, cart_cmd.y, cart_cmd.z};
            float current_q[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};

            if (TrajectoryPlanner_StartCtraj(target_xyz, cart_cmd.duration, current_q)) {
                for (int i = 0; i < 3; i++) {
                    motors[i]->state = TRAJ;
                }
                status_msg.data = 1; // 1 = Ejecutando
                rcl_publish(&status_publisher, &status_msg, NULL);
            } else {
                status_msg.data = -1; // -1 = Error / Inalcanzable
                rcl_publish(&status_publisher, &status_msg, NULL);
            }
        }

        // 3. Ejecucion del planificador con fase de asentamiento final a 100 Hz
        if (g_traj_planner.is_active) {
            float current_q[3] = {motor1.currentAngle, motor2.currentAngle, motor3.currentAngle};
            bool in_progress = TrajectoryPlanner_Update(CONTROL_LOOP_DT_SEC, q_cmd, qd_cmd, current_q);

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
                status_msg.data = 2; // 2 = Completado
                rcl_publish(&status_publisher, &status_msg, NULL);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MX_Tasks_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    mControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &mControlTask_attributes);
}

#include <unistd.h>

int usleep(useconds_t usec) {
    uint32_t ms = usec / 1000;
    if (ms == 0) {
        uint32_t loops = (uint32_t)((usec * (SystemCoreClock / 1000000UL)) / 4);
        for (volatile uint32_t i = 0; i < loops; i++) {
            __NOP();
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    return 0;
}
