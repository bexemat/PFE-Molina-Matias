/**
 * @file microros_callbacks.h
 * @brief Callbacks para tópicos y temporizadores en el entorno micro-ROS.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Despacha los mensajes entrantes hacia las estructuras del firmware y encola
 * comandos para su procesamiento determinista en el lazo de control a 100 Hz.
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
 * @brief Callback para la suscripción de trayectorias cartesianas (/microROS/cmd).
 */
void cmd_callback(const void * msgin);

/**
 * @brief Callback para el comando de homing (/microROS/homing).
 */
void homing_callback(const void * msgin);

/**
 * @brief Callback para el encendido y apagado del electroimán (/microROS/electroiman).
 */
void electromagnet_callback(const void * msgin);

/**
 * @brief Callback prioritario para la parada de emergencia (/microROS/emergency_stop).
 */
void estop_callback(const void * msgin);

/**
 * @brief Callback para la solicitud puntual de telemetría angular.
 */
void request_angles_callback(const void * msgin);

/**
 * @brief Callback periódico (40 Hz): publica los ángulos articulares actuales en /microROS/angles.
 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

/**
 * @brief Callback para comandos articulares punto a punto directos (/microROS/p2p_cmd).
 */
void p2p_cmd_callback(const void * msgin);

#ifdef __cplusplus
}
#endif

#endif /* INC_MICROROS_CALLBACKS_H_ */
