/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Punto de entrada de la aplicación, configuración HAL y ciclo de vida RTOS.
  * @author         : Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
  * @date           : 2026
  *
  * @details Inicializa los buses y periféricos de la placa STMicroelectronics NUCLEO-F446RE:
  *  - **Reloj del Sistema:** PLL a 180 MHz alimentado desde oscilador interno HSI (16 MHz)[cite: 22].
  *  - **Generación de Pasos:** Temporizadores hardware TIM13 (Q1), TIM2 (Q2) y TIM3 (Q3) en modo PWM[cite: 22].
  *  - **Adquisición Sensorial:** I2C1 a 400 kHz (Fast-Mode) y temporizador periódico TIM7 para muestreo de encoders[cite: 22].
  *  - **Comunicaciones Serie:** USART2 con DMA para la pila micro-ROS y USART3 para telemetría de depuración printf[cite: 22].
  *  - **Planificador de Tareas:** Despliegue de defaultTask (micro-ROS) y mControlTask (Lazo cinemático a 100 Hz)[cite: 22].
  *
  * @note Estructura 100% compatible con STM32CubeMX: Respeta estrictamente los bloques
  *       USER CODE BEGIN / USER CODE END para garantizar que la regeneración del archivo
  *       Firmware_V_1.0.ioc no sobrescriba las implementaciones de control ni de RTOS[cite: 22].
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motors.h"
#include "as5600_tca9548.h"
#include "electromagnet.h"
#include "microros_callbacks.h"
#include "tasks.h"
#include "kinematics.h"
#include "trajectory_planner.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4, // 8 KB para micro-ROS
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mControlTask */
osThreadId_t mControlTaskHandle;
const osThreadAttr_t mControlTask_attributes = {
  .name = "mControlTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* USER CODE BEGIN PV */
/**
 * @brief Cola de mensajes para transferir consignas cartesianas (X, Y, Z, T) entre tareas.
 * @details Desacopla la recepción asíncrona en defaultTask del lazo determinista a 100 Hz en mControlTask[cite: 22].
 */
osMessageQueueId_t mid_PositionQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/**
 * @brief Rutina de interrupción de temporizador para verificar la finalización de rampas P2P.
 * @param[in,out] m Puntero al descriptor del motor asociado al temporizador interviniente.
 */
static void Handle_Motor_Timer_Interrupt(Motor *m);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Punto de entrada principal del microcontrolador.
  * @retval int Código de retorno (nunca alcanzado tras el inicio del planificador de RTOS).
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset de todos los periféricos, inicialización de memoria Flash y SysTick */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configuración del árbol de reloj del sistema (180 MHz) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Inicialización de controladores y periféricos mapeados por hardware */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  /* Inicialización del estado inicial de los sensores magnéticos absolutos (Lectura bloqueante) */
  motor1.currentAngle = readAngle_AS5600(1);
  motor2.currentAngle = readAngle_AS5600(2);
  motor3.currentAngle = readAngle_AS5600(3);
  electromagnetOn(false);

  /* Inicio del temporizador TIM7 para disparar la máquina de adquisición no bloqueante de I2C */
  HAL_TIM_Base_Start_IT(&htim7);

  /* Detención automática de los contadores PWM durante pausas de depuración por hardware */
  __HAL_DBGMCU_FREEZE_TIM13();
  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_TIM3();

  printf("System ready.\r\n");
  printf("Angle1 = %.2f ---- Angle2 = %.2f ---- Angle3 = %.2f\r\n",
          motor1.currentAngle, motor2.currentAngle, motor3.currentAngle);
  /* USER CODE END 2 */

  /* Inicialización del kernel de CMSIS-RTOS V2 */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Mutexes del sistema (reservado) */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Semáforos binarios y contadores (reservado) */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* Temporizadores software de FreeRTOS (reservado) */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Instanciación de la cola para comandos cartesianos de trayectoria */
  mid_PositionQueue = osMessageQueueNew(4U, sizeof(CARTESIAN_CMD_t), NULL);
  if (mid_PositionQueue == NULL) {
    printf("Error: Could not create mid_PositionQueue.\r\n");
  }
  /* USER CODE END RTOS_QUEUES */

  /* Creación de tareas concurrentes */
  /* Tarea de comunicación micro-ROS (prioridad normal) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* Tarea de control cinemático determinista a 100 Hz (alta prioridad) */
  mControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &mControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Hilos de usuario adicionales (reservado) */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Banderas de eventos (reservado) */
  /* USER CODE END RTOS_EVENTS */

  /* Arranque del scheduler de FreeRTOS */
  osKernelStart();

  /* Bucle infinito de trampa por si el kernel cede el control */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Configura el árbol de reloj del sistema (System Clock Tree).
  * @details Fuente HSI (16 MHz) -> PLL (M=8, N=180, P=2) -> SYSCLK a 180 MHz[cite: 22].
  *          Buses periféricos: AHB = 180 MHz, APB1 = 45 MHz (Timers a 90 MHz), APB2 = 90 MHz[cite: 22].
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configuración del regulador interno de voltaje a máxima frecuencia (Scale 1) */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configuración del oscilador interno HSI y multiplicación por PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Inicialización de los relojes de buses CPU, AHB y APB */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicializa el periférico I2C1 para el bus de adquisición de encoders (Fast Mode 400 kHz).
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicializa el temporizador de 32 bits TIM2 para el canal PWM del Motor 2 (Hombro).
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief Inicializa el temporizador de 16 bits TIM3 para el canal PWM del Motor 3 (Muñeca).
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief Inicializa el temporizador básico TIM7 como base de tiempo de adquisición sensorial.
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicializa el temporizador de 16 bits TIM13 para el canal PWM del Motor 1 (Cintura).
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim13);
}

/**
  * @brief Inicializa el puerto USART2 a 115200 bps para el canal de transporte serie de micro-ROS.
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicializa el puerto USART3 a 115200 bps hacia el ST-LINK VCP para depuración (printf).
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicializa el controlador de acceso directo a memoria (DMA) para USART2.
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configuración de interrupciones para RX (Stream 5) y TX (Stream 6) */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/**
  * @brief Configura los puertos y pines GPIO de propósito general (señales DIR, MOSFET y pulsador).
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /* Pulsador de usuario azul (B1) */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Salidas de potencia del solenoide (PC3) y LED testigo (PC7) */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Salidas de dirección para drivers DRV8825: DIR1 (PA5) y DIR3 (PA8) */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Salida de dirección: DIR2 (PB10) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Entrada auxiliar PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
 * @brief Redirección de caracteres para la salida estándar stdout/printf hacia el puerto USART3.
 *
 * @param[in] ch Byte/carácter a transmitir.
 * @return int Carácter transmitido.
 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1U, HAL_MAX_DELAY);
  return ch;
}

/**
 * @brief Evalúa la condición de finalización de un eje durante movimientos punto a punto (P2P).
 *
 * @details Comprueba si la lectura del encoder magnético ha ingresado en la banda de tolerancia
 * (ANGLE_TOLERANCE) respecto al ángulo meta. Al alcanzar la meta o ante fallo de hardware, detiene
 * las interrupciones del timer, apaga la modulación PWM y commuta el actuador al estado IDLE[cite: 22].
 *
 * @param[in,out] m Puntero al descriptor del motor a inspeccionar[cite: 22].
 */
static void Handle_Motor_Timer_Interrupt(Motor *m) {
  switch (m->state) {
    case HOMING:
    case P2P: {
      bool target_reached = false;

      if (m->dir == GPIO_PIN_SET) {
        if (m->currentAngle >= (m->targetAngle - ANGLE_TOLERANCE)) {
          target_reached = true;
        }
      } else {
        if (m->currentAngle <= (m->targetAngle + ANGLE_TOLERANCE)) {
          target_reached = true;
        }
      }

      if (target_reached) {
        m->state = IDLE;
        HAL_TIM_Base_Stop_IT(m->timer);
        HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
      }
      break;
    }

    case MOTOR_ERROR:
      HAL_TIM_Base_Stop_IT(m->timer);
      HAL_TIM_PWM_Stop(m->timer, m->timerChannel);
      break;

    default:
      break;
  }
}

/**
 * @brief Callback global del HAL al completarse el periodo de cualquiera de los temporizadores activos.
 *
 * @details Distribuye las interrupciones periódicas:
 *  - **TIM13, TIM2, TIM3:** Disparan el chequeo de llegada para los motores 1, 2 y 3 respectivamente[cite: 22].
 *  - **TIM7:** Si el bus se encuentra en reposo (I2C_IDLE), inicia la lectura del siguiente encoder
 *    en un esquema secuencial no bloqueante tipo Round-Robin (1 -> 2 -> 3 -> 1)[cite: 22].
 *  - **TIM1:** Incrementa el contador de tiempo base (uwTick) del HAL de ST[cite: 22].
 *
 * @param[in] htim Puntero al manejador del temporizador que disparó la interrupción[cite: 22].
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t sensor_index = 1;

  if (htim->Instance == TIM13) {
    Handle_Motor_Timer_Interrupt(&motor1);
  }
  else if (htim->Instance == TIM2) {
    Handle_Motor_Timer_Interrupt(&motor2);
  }
  else if (htim->Instance == TIM3) {
    Handle_Motor_Timer_Interrupt(&motor3);
  }
  else if (htim->Instance == TIM7) {
    if (i2c_state == I2C_IDLE) {
      AS5600_StartRead_IT(sensor_index);
      sensor_index++;
      if (sensor_index > 3U) sensor_index = 1U;
    }
  }

  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
}
/* USER CODE END 4 */

/**
  * @brief  Manejador de excepciones críticas de hardware o inicialización fallida.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
