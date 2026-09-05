/**
 * @file electromagnet.c
 * @brief Implementación del driver de control para el actuador magnético del efector final.
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Gestiona la conmutación de la etapa de potencia basada en un transistor MOSFET de canal N
 * para energizar la bobina del electroimán/solenoide.
 *
 * @note Consideraciones de hardware y polaridad:
 *  - La compuerta (Gate) del MOSFET se activa con lógica invertida en esta placa:
 *      * GPIO_PIN_RESET (0V en salida lógica): Conducción activa del MOSFET -> Electroimán energizado[cite: 20].
 *      * GPIO_PIN_SET   (3.3V en salida lógica): Corte del MOSFET -> Electroimán desenergizado[cite: 20].
 *  - En paralelo, se conmuta un LED testigo montado en el chasis (LED_EMAGNET) con polaridad directa
 *    (GPIO_PIN_SET = encendido) para brindar confirmación visual inmediata al operario[cite: 20].
 */

#include "electromagnet.h"
#include "main.h"

/**
 * @brief Conmuta el estado de sujeción electromagnética y la señalización lumínica asociada.
 *
 * @details Aplica los niveles lógicos correspondientes sobre los puertos GPIO definidos en main.h:
 *  - Pin de compuerta: ELECTROMAGNET_PIN en ELECTROMAGNET_GPIO (PC3)[cite: 20, 22].
 *  - Pin de LED testigo: LED_EMAGNET_PIN en LED_EMAGNET_GPIO (PC7)[cite: 20, 22].
 *
 * @param[in] turn_on true para energizar la bobina y capturar la pieza; false para cortar
 *                    la excitación y liberar el objeto por gravedad[cite: 20].
 *
 * @note Esta operación es reentrante y determinista (O(1)), ejecutándose en tiempo constante.
 */
void electromagnetOn(bool turn_on) {
    if (turn_on) {
        /* Conducción activa del MOSFET (lógica baja) y encendido del LED de guardia */
        HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_SET);
    } else {
        /* Corte del MOSFET (lógica alta) y apagado del LED de guardia */
        HAL_GPIO_WritePin(ELECTROMAGNET_GPIO, ELECTROMAGNET_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_EMAGNET_GPIO, LED_EMAGNET_PIN, GPIO_PIN_RESET);
    }
}
