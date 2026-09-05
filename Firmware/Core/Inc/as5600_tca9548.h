/**
 * @file as5600_tca9548.h
 * @brief Driver para la lectura de encoders magnéticos AS5600 a través del multiplexor TCA9548A.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Controla el direccionamiento sobre el bus I2C a 400 kHz y la adquisición
 * de mediciones angulares de 12 bits para la retroalimentación de posición de los 3 ejes.
 */

#ifndef INC_AS5600_TCA9548_H_
#define INC_AS5600_TCA9548_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Estados para la máquina de adquisición no bloqueante sobre el bus I2C.
 */
typedef enum {
    I2C_IDLE = 0,       /**< Bus libre de transferencias */
    I2C_MUX_TX,         /**< Transmisión del byte de selección de canal al TCA9548A */
    I2C_AS5600_RX       /**< Recepción del registro angular de 12 bits desde el AS5600 */
} I2C_State_t;

extern volatile I2C_State_t i2c_state;
extern volatile int i2c_last_ret;

/**
 * @brief Configura el canal activo dentro del multiplexor TCA9548A.
 *
 * @param[in] channel Canal de conmutación I2C deseado (0 a 7).
 * @return HAL_StatusTypeDef HAL_OK si el periférico respondió adecuadamente.
 */
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);

/**
 * @brief Lectura bloqueante directa del ángulo mecánico absoluto en grados.
 *
 * @param[in] motor Índice del actuador a muestrear (0: Cintura, 1: Hombro, 2: Muñeca).
 * @return float Posición angular absoluta normalizada [0.0° a 360.0°]. Retorna valor negativo ante fallas de bus.
 */
float readAngle_AS5600(int motor);

/**
 * @brief Filtro pasa-bajos de primer orden para atenuar ruido de alta frecuencia en el sensor.
 *
 * @param[in] angleReaded Lectura angular cruda más reciente [°].
 * @param[in] lastAngle   Lectura angular filtrada del ciclo anterior [°].
 * @return float Ángulo filtrado resultante [°].
 */
float filterAngle(float angleReaded, float lastAngle);

/**
 * @brief Inicia la secuencia de lectura asíncrona mediante interrupciones de temporizador.
 *
 * @param[in] motor Índice del motor cuyo canal se interroga.
 */
void AS5600_StartRead_IT(int motor);

#ifdef __cplusplus
}
#endif

#endif /* INC_AS5600_TCA9548_H_ */
