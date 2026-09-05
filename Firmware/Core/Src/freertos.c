/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : Manejadores de ganchos (hooks) y supervisión del kernel FreeRTOS.
  * @author         : Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
  * @date           : 2026
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
/**
 * @brief Gancho de seguridad invocado por el kernel ante desbordamiento de pila (Stack Overflow).
 *
 * @details Se activa automáticamente cuando el planificador de FreeRTOS (con configCHECK_FOR_STACK_OVERFLOW = 2)
 * detecta que el puntero de pila de una tarea ha sobreescrito el patrón canario límite en su bloque TCB.
 *
 * Comportamiento determinista ante fallo:
 *  1. Registra por la consola serie (USART3 / VCP) el nombre textual de la tarea que excedió su stack[cite: 21].
 *  2. Inhabilita globalmente las interrupciones del microcontrolador (taskDISABLE_INTERRUPTS) para evitar
 *     que las ISRs o el conmutador de contexto operen sobre memoria corrompida[cite: 21].
 *  3. Entra en un bucle infinito for(;;) actuando como trampa de hardware (Hard Trap) para preservar
 *     el estado de los registros y facilitar la inspección post-mortem con depurador SWD/JTAG[cite: 21].
 *
 * @param[in] xTask      Manejador (Handle) de la tarea causante del desbordamiento[cite: 21].
 * @param[in] pcTaskName Cadena terminada en null con el nombre asignado a la tarea desbordada[cite: 21].
 *
 * @warning Bajo ninguna circunstancia se debe intentar retornar de este hook, ya que el contexto
 *          de ejecución del sistema operativo se encuentra irrecuperablemente comprometido.
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    printf("[CRITICAL ERROR RTOS] Stack overflow detectado en la tarea: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;) {
        /* Bucle trampa de seguridad para inspección con depurador */
    }
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
