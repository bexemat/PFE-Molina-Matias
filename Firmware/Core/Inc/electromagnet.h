#ifndef INC_ELECTROMAGNET_H_
#define INC_ELECTROMAGNET_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Conmuta el estado de conduccion del MOSFET del electroiman y el LED testigo.
 * @param[in] turn_on true para energizar, false para desenergizar.
 */
void electromagnetOn(bool turn_on);

#ifdef __cplusplus
}
#endif

#endif /* INC_ELECTROMAGNET_H_ */
