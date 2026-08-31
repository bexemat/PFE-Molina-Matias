/* Src/as5600_tca9548.c */
#include "as5600_tca9548.h"
#include "main.h"
#include "motors.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

volatile I2C_State_t i2c_state = I2C_IDLE;
volatile int i2c_last_ret = 0;
volatile int current_motor = 0;
static uint8_t mux_data;
static uint8_t as5600_buf[2];

HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel) {
    uint8_t data = 1 << channel;
    return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR, &data, 1, HAL_MAX_DELAY);
}

float readAngle_AS5600(int motor) {
    uint8_t data[2];
    bool invert;

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

    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH, I2C_MEMADD_SIZE_8BIT, &data[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_LOW,  I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

    uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
    uint16_t rawAngle = angle & 0x0FFF;

    if (invert) {
            rawAngle = (4096 - rawAngle) & 0x0FFF;
        }

        float currentAngle = (rawAngle * 360.0f) / 4096.0f;

        // --- NUEVA CALIBRACIÓN DE OFFSET (Q2) ---
        if (motor == 2) {
            currentAngle += 180.00f;
            // Si al sumar nos pasamos de una vuelta completa, lo normalizamos
            if (currentAngle >= 360.0f) {
                currentAngle -= 360.0f;
            }
        }
        // ----------------------------------------

        // Normalización estándar de -180 a 180 para todos los motores
        if (currentAngle > 180.0f) {
            currentAngle -= 360.0f;
        }

        return currentAngle;
    }

float filterAngle(float angleReaded, float lastAngle) {
    if (fabsf(angleReaded - lastAngle) > ANGLE_REJECT_DEG) {
        return lastAngle;
    }
    return angleReaded;
}

void AS5600_StartRead_IT(int motor) {
    if (i2c_state != I2C_IDLE) return;

    current_motor = motor;

    if (motor == 1)      mux_data = 1 << 2;
    else if (motor == 2) mux_data = 1 << 1;
    else if (motor == 3) mux_data = 1 << 0;
    else return;

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(&hi2c1, TCA9548A_ADDR, &mux_data, 1);
    if (ret == HAL_OK) {
        i2c_state = I2C_MUX_TX;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) return;

    if (i2c_state == I2C_MUX_TX) {
        HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(&hi2c1, AS5600_ADDR, ANGLE_REG_HIGH, I2C_MEMADD_SIZE_8BIT, as5600_buf, 2);
        if (ret == HAL_OK) {
            i2c_state = I2C_AS5600_RX;
        } else {
            i2c_last_ret = (int)ret;
            i2c_state = I2C_IDLE;
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) return;

    if (i2c_state == I2C_AS5600_RX) {
        uint16_t raw = ((uint16_t)as5600_buf[0] << 8) | as5600_buf[1];
        raw &= 0x0FFF;
        bool invert;

        switch (current_motor) {
            case 1:
            case 3: invert = true; break;
            case 2: invert = false; break;
            default: return;
        }

        if (invert) raw = (4096 - raw) & 0x0FFF;

                float currentAngle = (raw * 360.0f) / 4096.0f;

                // --- NUEVA CALIBRACIÓN DE OFFSET (Q2) ---
                if (current_motor == 2) {
                    currentAngle += 180.00f;
                    if (currentAngle >= 360.0f) {
                        currentAngle -= 360.0f;
                    }
                }
                // ----------------------------------------

                if (currentAngle > 180.0f) {
                    currentAngle -= 360.0f;
                }


        switch (current_motor) {
            case 1: motor1.currentAngle = filterAngle(currentAngle, motor1.currentAngle); break;
            case 2: motor2.currentAngle = filterAngle(currentAngle, motor2.currentAngle); break;
            case 3: motor3.currentAngle = filterAngle(currentAngle, motor3.currentAngle); break;
            default: return;
        }

        i2c_state = I2C_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) return;
    i2c_last_ret = (int)hi2c->ErrorCode;
    i2c_state = I2C_IDLE;
}
