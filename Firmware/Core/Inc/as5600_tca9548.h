/* Inc/as5600_tca9548.h */
#ifndef INC_AS5600_TCA9548_H_
#define INC_AS5600_TCA9548_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum {
    I2C_IDLE,
    I2C_MUX_TX,
    I2C_AS5600_RX
} I2C_State_t;

extern volatile I2C_State_t i2c_state;
extern volatile int i2c_last_ret;

HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);
float readAngle_AS5600(int motor);
float filterAngle(float angleReaded, float lastAngle);
void AS5600_StartRead_IT(int motor);

#endif /* INC_AS5600_TCA9548_H_ */
