/* Src/microros_callbacks.c */
#include "microros_callbacks.h"
#include "electromagnet.h"
#include "motors.h"
#include "trajectory_planner.h"
#include "cmsis_os.h"
#include <stdio.h>

extern osMessageQueueId_t mid_PositionQueue;
extern rcl_publisher_t publisher;
extern geometry_msgs__msg__Point tim_msg;

void cmd_callback(const void * msgin) {
    const extra_interfaces__msg__Trama * msg = (const extra_interfaces__msg__Trama *)msgin;

    CARTESIAN_CMD_t cmd = {
        .x = (float)msg->q[0],
        .y = (float)msg->q[1],
        .z = (float)msg->q[2],
        .duration = (float)msg->t_total
    };

    osMessageQueuePut(mid_PositionQueue, &cmd, 0U, 0U);
}

void p2p_cmd_callback(const void * msgin) {
    const geometry_msgs__msg__Point *msg = (const geometry_msgs__msg__Point *)msgin;
    float targets[3] = {(float)msg->x, (float)msg->y, (float)msg->z};
    startSynchronizedP2PMovement(targets);
}

void homing_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    if (msg->data) {
        doAllHoming();
    }
}

void electromagnet_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    electromagnetOn(msg->data);
}

void estop_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    bool estop_engaged = msg->data;

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

void request_angles_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        tim_msg.x = (double)motor1.currentAngle;
        tim_msg.y = (double)motor2.currentAngle;
        tim_msg.z = (double)motor3.currentAngle;
        rcl_publish(&publisher, &tim_msg, NULL);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    tim_msg.x = (double)motor1.currentAngle;
    tim_msg.y = (double)motor2.currentAngle;
    tim_msg.z = (double)motor3.currentAngle;

    if (timer != NULL) {
        rcl_publish(&publisher, &tim_msg, NULL);
    }
}
