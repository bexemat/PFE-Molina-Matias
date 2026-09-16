/**
 * @file microros_callbacks.h
 * @brief Declaración de callbacks asíncronos para la capa de middleware micro-ROS.
 * @author Matías Exequiel Molina <matimolina123@gmail.com>
 * @date 2026
 *
 * @details Despacha los mensajes entrantes desde ROS 2 hacia los descriptores del firmware,
 * administra el encolado no bloqueante de metas cartesianas y comanda la parada de emergencia lógica.
 */

#ifndef INC_MICROROS_CALLBACKS_H_
#define INC_MICROROS_CALLBACKS_H_

#include "main.h"
#include "tasks.h"
#include <rcl/rcl.h>

#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/point.h>
#include <extra_interfaces/msg/trama.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback de recepción de metas cartesianas (/microROS/cmd).
 */
void cmd_callback(const void * msgin);

/**
 * @brief Callback para inicio de la rutina de homing articular (/microROS/homing).
 */
void homing_callback(const void * msgin);

/**
 * @brief Callback para accionamiento del efector final magnético (/microROS/electroiman).
 */
void electromagnet_callback(const void * msgin);

/**
 * @brief Callback prioritario para la parada de emergencia por software (/microROS/emergency_stop).
 * @note Opera como parada lógica por corte de trenes PWM y aborto de trayectoria en MCU.
 */
void estop_callback(const void * msgin);

/**
 * @brief Callback para atender solicitudes discretas de telemetría angular (/microROS/request_current_angles).
 */
void request_angles_callback(const void * msgin);

/**
 * @brief Callback periódico del temporizador micro-ROS (40 Hz): publica telemetría en /microROS/angles.
 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

/**
 * @brief Callback para comandos directos en espacio articular (/microROS/p2p_cmd).
 */
void p2p_cmd_callback(const void * msgin);

#ifdef __cplusplus
}
#endif

#endif /* INC_MICROROS_CALLBACKS_H_ */
