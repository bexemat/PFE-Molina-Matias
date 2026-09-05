/**
 * @file tasks.h
 * @brief Declaración de hilos de ejecución FreeRTOS, buffers y adaptadores de transporte micro-ROS.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * PERIODOS TEMPORALES Y FRECUENCIAS DE EJECUCIÓN
 * ========================================================================= */
#define CONTROL_LOOP_PERIOD_MS   (10)   /**< Periodo del lazo de cinemática y control mControlTask (100 Hz) */
#define TELEMETRY_TIMER_MS       (25)   /**< Periodo del timer micro-ROS para publicación de ángulos (40 Hz) */

/**
 * @brief Consigna cartesiana estructurada recibida desde la capa de supervisión.
 */
typedef struct {
    float x;            /**< Posición cartesiana longitudinal X [mm] */
    float y;            /**< Posición cartesiana transversal Y [mm] */
    float z;            /**< Posición cartesiana vertical Z [mm] */
    float duration;     /**< Duración deseada para completar el trayecto [s] */
} CARTESIAN_CMD_t;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t mControlTaskHandle;

extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t mControlTask_attributes;

/**
 * @brief Tarea de prioridad normal: gestión del ejecutor de micro-ROS y callbacks.
 */
void StartDefaultTask(void *argument);

/**
 * @brief Tarea de alta prioridad: lazo determinista a 100 Hz de control cinemático y perfiles PWM.
 */
void StartMotorControlTask(void *argument);

/**
 * @brief Configuración e instanciación de objetos del sistema operativo FreeRTOS.
 */
void MX_Tasks_Init(void);

/* =========================================================================
 * ADAPTADORES DE TRANSPORTE PERSONALIZADO PARA MICRO-ROS (UART DMA)
 * ========================================================================= */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

/* Hooks de gestión de memoria dinámica para middleware micro-ROS */
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#ifdef __cplusplus
}
#endif

#endif /* INC_TASKS_H_ */
