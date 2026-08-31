/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <rcl/rcl.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    double x;
    double y;
    double z;
} CARTESIAN_POS_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim13;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* Mapeo de Shield CNC y Drivers */
#define DIR1_GPIO           GPIOA
#define DIR1_PIN            GPIO_PIN_5

#define DIR2_GPIO           GPIOB
#define DIR2_PIN            GPIO_PIN_10

#define DIR3_GPIO           GPIOA
#define DIR3_PIN            GPIO_PIN_8

#define LED_EMAGNET_GPIO    GPIOC
#define LED_EMAGNET_PIN     GPIO_PIN_7

#define ELECTROMAGNET_GPIO  GPIOC
#define ELECTROMAGNET_PIN   GPIO_PIN_3

/* Multiplexor I2C y Sensor AS5600 */
#define TCA9548A_ADDR       (0x70 << 1)
#define AS5600_ADDR         (0x36 << 1)
#define ANGLE_REG_HIGH      0x0E
#define ANGLE_REG_LOW       0x0F

/* Parámetros de Control y Tolerancias */
#define ANGLE_TOLERANCE     0.25f
#define ANGLE_REJECT_DEG    180.0f
#define KP_POS              5.0f
#define KP_APPROX           5.0f

int __io_putchar(int ch);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
