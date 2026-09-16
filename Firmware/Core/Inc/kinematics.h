/**
 * @file kinematics.h
 * @brief Modelado cinemático analítico para manipulador serial de 3 GDL con mecanismo de paralelogramo.
 * @author Matías Exequiel Molina <matimolina123@gmail.com>
 * @date 2026
 *
 * @details Implementa la cinemática directa (FK) y la cinemática inversa analítica (IK)
 * mediante formulación trigonométrica cerrada en el semiplano sagital r-z'.
 * Resuelve la cinemática considerando el desacople inducido por el mecanismo de cuatro barras,
 * el cual mantiene pasivamente la orientación horizontal del efector final y elimina
 * la necesidad de articulaciones virtuales en la convención Denavit-Hartenberg.
 *
 * @note Dimensiones métricas en milímetros [mm] y coordenadas articulares en grados
 * sexagesimales [°], optimizadas para cálculo en simple precisión con la FPU del STM32F446RE.
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * PARÁMETROS GEOMÉTRICOS Y DIMENSIONES NOMINALES CAD [mm]
 * ========================================================================= */
#define KIN_L1_MM           (77.00f)    /**< Altura de la base al centro del eje de hombro [mm] */
#define KIN_L2_MM           (135.00f)   /**< Longitud del brazo principal (hombro a codo) [mm] */
#define KIN_L3_MM           (147.00f)   /**< Longitud del antebrazo / paralelogramo (codo a muñeca) [mm] */
#define KIN_O_R_MM          (25.49f)    /**< Desfase radial constructivo del efector final [mm] */
#define KIN_O_Z_MM          (40.87f)    /**< Desfase vertical constructivo del efector final [mm] */

/* Alias de compatibilidad hacia atrás */
#define KIN_L1              KIN_L1_MM
#define KIN_L2              KIN_L2_MM
#define KIN_L3              KIN_L3_MM
#define KIN_O_R             KIN_O_R_MM
#define KIN_O_Z             KIN_O_Z_MM

/* Constantes matemáticas de conversión angular */
#define RAD_TO_DEG          (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD_F        (3.14159265358979323846f / 180.0f)

/* =========================================================================
 * LÍMITES MECÁNICOS Y DE SEGURIDAD ARTICULAR FÍSICOS [°]
 * ========================================================================= */
#define Q1_MIN_DEG          (-90.0f)    /**< Límite inferior cintura (rotación azimutal base) */
#define Q1_MAX_DEG          (90.0f)     /**< Límite superior cintura (rotación azimutal base) */
#define Q2_MIN_DEG          (20.0f)     /**< Límite inferior hombro (corregido: evita colisión mecánica) */
#define Q2_MAX_DEG          (157.0f)    /**< Límite superior hombro (elevación máxima del brazo) */
#define Q3_MIN_DEG          (-75.0f)    /**< Límite inferior muñeca / paralelogramo */
#define Q3_MAX_DEG          (25.0f)     /**< Límite superior muñeca / paralelogramo */

/**
 * @brief Resuelve la cinemática directa analítica (Forward Kinematics).
 * @details Transforma las coordenadas generalizadas [Q1, Q2, Q3] a la posición cartesiana
 * tridimensional [X, Y, Z] del centro del efector final magnético respecto al origen de la base.
 *
 * @param[in]  q_deg   Arreglo de 3 elementos con las coordenadas articulares [Q1, Q2, Q3] en [°].
 * @param[out] pos_mm  Arreglo de 3 elementos donde se retornan las coordenadas cartesianas [X, Y, Z] en [mm].
 *
 * @return true si el cálculo es matemáticamente admisible; false ante error numérico.
 */
bool Kinematics_FK(const float q_deg[3], float pos_mm[3]);

/**
 * @brief Resuelve la cinemática inversa analítica geométrica (Inverse Kinematics).
 * @details Determina los ángulos articulares requeridos a partir de una posición cartesiana meta [X, Y, Z],
 * forzando de forma cerrada la rama geométrica de configuración "codo arriba" (elbow-up) y aislando singularidades.
 *
 * @param[in]  pos_mm  Coordenadas cartesianas objetivo [X, Y, Z] en [mm].
 * @param[out] q_deg   Arreglo de salida donde se depositan los ángulos articulares [Q1, Q2, Q3] en [°].
 *
 * @return true si la pose es alcanzable dentro del workspace admisible y respeta topes mecánicos;
 *         false si la posición es inalcanzable, singular o viola las cotas articulares.
 */
bool Kinematics_IK(const float pos_mm[3], float q_deg[3]);

/**
 * @brief Evalúa si un vector de ángulos articulares respeta los límites mecánicos del manipulador.
 *
 * @param[in] q_deg Arreglo de 3 elementos con los ángulos articulares a evaluar [Q1, Q2, Q3] en [°].
 * @return true si las 3 articulaciones están dentro de las cotas operativas; false si al menos una excede su cota.
 */
bool Kinematics_CheckJointLimits(const float q_deg[3]);

#ifdef __cplusplus
}
#endif

#endif /* INC_KINEMATICS_H_ */
