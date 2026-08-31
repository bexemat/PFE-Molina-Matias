/* Src/electromagnet.c */
#include "electromagnet.h"
#include "main.h"

void electromagnetOn(bool turn_on) {
    if (turn_on) {
        HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_RESET);
    }
}
