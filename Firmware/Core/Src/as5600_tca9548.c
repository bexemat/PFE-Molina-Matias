/**
 * @file as5600_tca9548.c
 * @brief Implementación del driver para encoders magnéticos AS5600 vía multiplexor I2C TCA9548A.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Administra el bus I2C1 a 400 kHz con el siguiente diseño:
 *  - **Lectura bloqueante (readAngle_AS5600):** Utilizada durante el arranque del sistema en main()
 *    para inicializar la pose articular antes de arrancar FreeRTOS.
 *  - **Lectura asíncrona no bloqueante (AS5600_StartRead_IT):** Disparada periódicamente desde
 *    la ISR de TIM7 para actualizar los ángulos articulares en segundo plano sin detener el lazo de control.
 */

#include "as5600_tca9548.h"
#include "main.h"
#include "motors.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/** @brief Estado actual de la máquina de estados no bloqueante I2C */
volatile I2C_State_t i2c_state = I2C_IDLE;

/** @brief Registro del último código de error devuelto por el bus I2C */
volatile int i2c_last_ret = 0;

/** @brief Índice del motor actualmente seleccionado en la secuencia de interrupción */
volatile int current_motor = 0;

static uint8_t mux_data;
static uint8_t as5600_buf[2];

/**
 * @brief Selecciona de forma síncrona el canal activo en el multiplexor I2C TCA9548A.
 *
 * @param[in] channel Canal numérico a habilitar (0 a 7).
 * @return HAL_StatusTypeDef HAL_OK si el byte fue recibido correctamente por el multiplexor.
 */
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel) {
    const uint8_t data = 1U << channel;
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, (uint8_t*)&data, 1U, HAL_MAX_DELAY);
}

/**
 * @brief Lectura bloqueante síncrona del ángulo articular absoluto de un motor.
 *
 * @details Realiza la conmutación de canal del multiplexor, solicita la lectura de los registros
 * 0x0E (MSB) y 0x0F (LSB) del AS5600, decodifica los 12 bits (0-4095), aplica inversión de giro
 * según la disposición mecánica y normaliza el rango al semiplano angular [-180.0°, +180.0°].
 *
 * @param[in] motor Índice del motor a interrogar (1: Cintura, 2: Hombro, 3: Muñeca).
 * @return float Ángulo mecánico absoluto en grados sexagesimales [°].
 */
float readAngle_AS5600(int motor) {
    uint8_t data[2];
    bool invert;

    /* Enrutamiento por canal físico del multiplexor y polaridad de montaje */
    if (motor == 2) {
        TCA9548A_SelectChannel(1);
        invert = false;
    } else if (motor == 3) {
        TCA9548A_SelectChannel(0);
        invert = true;
    } else if (motor == 1) {
        TCA9548A_SelectChannel(2);
        invert = true;
    } else {
        return 0.0f;
    }

    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH, I2C_MEMADD_SIZE_8BIT, &data[0], 1U, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_LOW,  I2C_MEMADD_SIZE_8BIT, &data[1], 1U, HAL_MAX_DELAY);

    const uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
    uint16_t rawAngle = angle & 0x0FFFU;

    if (invert) {
        rawAngle = (4096U - rawAngle) & 0x0FFFU;
    }

    float currentAngle = (rawAngle * 360.0f) / 4096.0f;

    /* Calibración mecánica específica para el sensor del hombro (Q2) */
    if (motor == 2) {
        currentAngle += 180.00f;
        if (currentAngle >= 360.0f) {
            currentAngle -= 360.0f;
        }
    }

    /* Normalización al semiplano simétrico [-180.0°, +180.0°] */
    if (currentAngle > 180.0f) {
        currentAngle -= 360.0f;
    }

    return currentAngle;
}

/**
 * @brief Filtro de rechazo a anomalías bruscas por ruido de comunicación o discontinuidad de cuadrante.
 *
 * @param[in] angleReaded Nueva lectura angular cruda [°].
 * @param[in] lastAngle   Lectura angular previamente validada [°].
 * @return float Valor aceptado: si el salto supera ANGLE_REJECT_DEG, se descarta y conserva lastAngle.
 */
float filterAngle(float angleReaded, float lastAngle) {
    if (fabsf(angleReaded - lastAngle) > ANGLE_REJECT_DEG) {
        return lastAngle;
    }
    return angleReaded;
}

/**
 * @brief Inicia la transferencia no bloqueante por interrupción para adquisición periódica de ángulos.
 *
 * @param[in] motor Índice del motor a interrogar (1, 2 o 3).
 */
void AS5600_StartRead_IT(int motor) {
    if (i2c_state != I2C_IDLE) {
        return;
    }

    current_motor = motor;

    if (motor == 1) {
        mux_data = 1U << 2;
    } else if (motor == 2) {
        mux_data = 1U << 1;
    } else if (motor == 3) {
        mux_data = 1U << 0;
    } else {
        return;
    }

    const HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(&hi2c1, TCA9548A_ADDR, &mux_data, 1U);
    if (ret == HAL_OK) {
        i2c_state = I2C_MUX_TX;
    }
}

/**
 * @brief Callback HAL al completar la transmisión del byte de conmutación al multiplexor.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return;
    }

    if (i2c_state == I2C_MUX_TX) {
        const HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH,
                                                          I2C_MEMADD_SIZE_8BIT, as5600_buf, 2U);
        if (ret == HAL_OK) {
            i2c_state = I2C_AS5600_RX;
        } else {
            i2c_last_ret = (int)ret;
            i2c_state = I2C_IDLE;
        }
    }
}

/**
 * @brief Callback HAL al recibir los 2 bytes angulares desde el encoder AS5600.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return;
    }

    if (i2c_state == I2C_AS5600_RX) {
        uint16_t raw = ((uint16_t)as5600_buf[0] << 8) | as5600_buf[1];
        raw &= 0x0FFFU;
        bool invert;

        switch (current_motor) {
            case 1:
            case 3:
                invert = true;
                break;
            case 2:
                invert = false;
                break;
            default:
                i2c_state = I2C_IDLE;
                return;
        }

        if (invert) {
            raw = (4096U - raw) & 0x0FFFU;
        }

        float currentAngle = (raw * 360.0f) / 4096.0f;

        if (current_motor == 2) {
            currentAngle += 180.00f;
            if (currentAngle >= 360.0f) {
                currentAngle -= 360.0f;
            }
        }

        if (currentAngle > 180.0f) {
            currentAngle -= 360.0f;
        }

        /* Actualización filtrada de la telemetría en el descriptor del actuador correspondiente */
        switch (current_motor) {
            case 1:
                motor1.currentAngle = filterAngle(currentAngle, motor1.currentAngle);
                break;
            case 2:
                motor2.currentAngle = filterAngle(currentAngle, motor2.currentAngle);
                break;
            case 3:
                motor3.currentAngle = filterAngle(currentAngle, motor3.currentAngle);
                break;
            default:
                break;
        }

        i2c_state = I2C_IDLE;
    }
}

/**
 * @brief Callback HAL de gestión de fallos de bus I2C.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return;
    }
    i2c_last_ret = (int)hi2c->ErrorCode;
    i2c_state = I2C_IDLE;
}
