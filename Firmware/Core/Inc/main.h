/**
 * @file main.h
 * @brief Cabecera principal del sistema de control embebido y mapeo de hardware.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Centraliza las definiciones globales del microcontrolador STM32F446RE,
 * incluyendo el mapeo de pines para el control de potencia (drivers DRV8825 y MOSFET del efector),
 * direcciones físicas sobre el bus I2C a 400 kHz, temporizadores para modulación PWM por hardware,
 * estructuras de datos cartesianas y macros de validación para la pila de cliente micro-ROS (RCLC).
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <rcl/rcl.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Estructura para almacenamiento de coordenadas cartesianas tridimensionales.
 * @note Utilizada para telemetría interna y cálculos de cinemática directa.
 */
typedef struct {
    double x;   /**< Posición en el eje longitudinal X [mm] */
    double y;   /**< Posición en el eje transversal Y [mm] */
    double z;   /**< Posición en el eje vertical Z [mm] */
} CARTESIAN_POS_t;

/* Exported constants --------------------------------------------------------*/

/** @brief Manejador del temporizador TIM2 para generación PWM del Motor 2 (Hombro / Q2). */
extern TIM_HandleTypeDef htim2;

/** @brief Manejador del temporizador TIM3 para generación PWM del Motor 3 (Muñeca / Q3). */
extern TIM_HandleTypeDef htim3;

/** @brief Manejador del temporizador TIM13 para generación PWM del Motor 1 (Cintura / Q1). */
extern TIM_HandleTypeDef htim13;

/* Exported macros -----------------------------------------------------------*/

/**
 * @brief Macro de validación estricta para llamadas al middleware micro-ROS (RCL).
 * @details Evalúa el código de retorno. Si no es RCL_RET_OK, imprime un reporte de diagnóstico
 * por consola con línea y código de error, y aborta la tarea FreeRTOS invocante de forma segura.
 * @param[in] fn Función o expresión RCL a evaluar.
 */
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

/**
 * @brief Macro de validación no bloqueante para funciones secundarias de micro-ROS.
 * @details Registra la advertencia por consola pero permite que la ejecución continúe sin abortar el hilo.
 * @param[in] fn Función o expresión RCL a evaluar.
 */
#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    } \
}

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Inicializa los puertos GPIO asignados a las salidas auxiliares de los temporizadores.
 * @param[in,out] htim Puntero al temporizador hardware configurado.
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 * @brief Manejador de fallas críticas de hardware o inicialización de periféricos HAL.
 * @details Bloquea la CPU en un bucle infinito para preservar la integridad del sistema mecánico.
 */
void Error_Handler(void);

/**
 * @brief Redirección de salida de caracteres (printf) hacia el puerto serie UART para depuración.
 * @param[in] ch Carácter a transmitir.
 * @return Carácter transmitido en caso de éxito.
 */
int __io_putchar(int ch);

/* Private defines -----------------------------------------------------------*/

/* --- Señales estándar de la placa NUCLEO-F446RE y depuración SWD --- */
#define B1_Pin                  GPIO_PIN_13
#define B1_GPIO_Port            GPIOC
#define USART_TX_Pin            GPIO_PIN_2
#define USART_TX_GPIO_Port      GPIOA
#define USART_RX_Pin            GPIO_PIN_3
#define USART_RX_GPIO_Port      GPIOA
#define TMS_Pin                 GPIO_PIN_13
#define TMS_GPIO_Port           GPIOA
#define TCK_Pin                 GPIO_PIN_14
#define TCK_GPIO_Port           GPIOA

/* --- Mapeo de salidas de dirección (DIR) para controladores paso a paso DRV8825 --- */
#define DIR1_GPIO               GPIOA           /**< Puerto GPIO de dirección: Motor 1 (Cintura) */
#define DIR1_PIN                GPIO_PIN_5      /**< Pin GPIO de dirección: Motor 1 (Cintura) */

#define DIR2_GPIO               GPIOB           /**< Puerto GPIO de dirección: Motor 2 (Hombro) */
#define DIR2_PIN                GPIO_PIN_10     /**< Pin GPIO de dirección: Motor 2 (Hombro) */

#define DIR3_GPIO               GPIOA           /**< Puerto GPIO de dirección: Motor 3 (Muñeca) */
#define DIR3_PIN                GPIO_PIN_8      /**< Pin GPIO de dirección: Motor 3 (Muñeca) */

/* --- Control de etapa de potencia del efector final (Electroimán) --- */
#define LED_EMAGNET_GPIO        GPIOC           /**< Puerto GPIO del LED testigo de activación del efector */
#define LED_EMAGNET_PIN         GPIO_PIN_7      /**< Pin GPIO del LED testigo de activación del efector */

#define ELECTROMAGNET_GPIO      GPIOC           /**< Puerto GPIO de compuerta hacia MOSFET de potencia */
#define ELECTROMAGNET_PIN       GPIO_PIN_3      /**< Pin GPIO de compuerta hacia MOSFET de potencia */

/* --- Direcciones I2C de periféricos en bus a 400 kHz (Desplazadas para HAL de ST) --- */
#define TCA9548A_ADDR           (0x70 << 1)     /**< Dirección en bus I2C del multiplexor TCA9548A */
#define AS5600_ADDR             (0x36 << 1)     /**< Dirección en bus I2C del encoder magnético AS5600 */
#define ANGLE_REG_HIGH          (0x0E)          /**< Dirección del registro de lectura angular alta (RAW ANGLE 11:8) */
#define ANGLE_REG_LOW           (0x0F)          /**< Dirección del registro de lectura angular baja (RAW ANGLE 7:0) */

/* --- Parámetros de control de lazo cerrado y tolerancias mecánicas --- */
#define ANGLE_TOLERANCE         (0.25f)         /**< Ventana de tolerancia angular para declarar posición alcanzada [°] */
#define ANGLE_REJECT_DEG        (180.0f)        /**< Umbral de variación brusca para rechazar lecturas anómalas del sensor [°] */
#define KP_POS                  (5.0f)          /**< Ganancia proporcional por defecto para control posicional articular */
#define KP_APPROX               (5.0f)          /**< Ganancia proporcional para la fase terminal de asentamiento */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
