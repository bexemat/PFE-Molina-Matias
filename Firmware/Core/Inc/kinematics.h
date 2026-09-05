/**
 * @file kinematics.h
 * @brief Modelado cinemático analítico para robot paralelo de 3 GDL (EEZYbotARM MK2).
 * @author Matías Exequiel Molina <ingenieria@uncuyo.edu.ar>
 * @date 2026
 *
 * @details Implementa la cinemática directa (FK) y la cinemática inversa (IK)
 * mediante un método trigonométrico directo en el plano r-z', adaptado a la morfología
 * de lazo cerrado (mecanismo de 4 barras). Elimina los eslabones y articulaciones virtuales
 * de la convención clásica Denavit-Hartenberg.
 *
 * @note Todas las dimensiones métricas están unificadas en milímetros [mm] y los
 * ángulos articulares en grados sexagesimales [°], con precisión simple para la FPU
 * del microcontrolador STM32F446RE.
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * PARÁMETROS GEOMÉTRICOS Y DIMENSIONES NOMINALES [mm]
 * ========================================================================= */
#define KIN_L1_MM           (77.00f)    /**< Altura de la base al centro del hombro [mm] */
#define KIN_L2_MM           (135.00f)   /**< Longitud del brazo principal (hombro a codo) [mm] */
#define KIN_L3_MM           (147.00f)   /**< Longitud del antebrazo (codo a muñeca) [mm] */
#define KIN_O_R_MM          (25.49f)    /**< Desfase radial del efector final [mm] */
#define KIN_O_Z_MM          (40.87f)    /**< Desfase vertical del efector final [mm] */

/* Alias de compatibilidad con código legacy */
#define KIN_L1              KIN_L1_MM
#define KIN_L2              KIN_L2_MM
#define KIN_L3              KIN_L3_MM
#define KIN_O_R             KIN_O_R_MM
#define KIN_O_Z             KIN_O_Z_MM

/* Constantes matemáticas de conversión angular */
#define RAD_TO_DEG          (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD_F        (3.14159265358979323846f / 180.0f)

/* Límites de carrera físicos y de seguridad articular [°] */
#define Q1_MIN_DEG          (-90.0f)    /**< Límite inferior cintura (rotación base) */
#define Q1_MAX_DEG          (90.0f)     /**< Límite superior cintura (rotación base) */
#define Q2_MIN_DEG          (40.0f)     /**< Límite inferior hombro (elevación brazo) */
#define Q2_MAX_DEG          (157.0f)    /**< Límite superior hombro (elevación brazo) */
#define Q3_MIN_DEG          (-75.0f)    /**< Límite inferior muñeca / antebrazo */
#define Q3_MAX_DEG          (25.0f)     /**< Límite superior muñeca / antebrazo */

/**
 * @brief Resuelve la cinemática directa (Forward Kinematics).
 * @details Proyecta los eslabones físicos en el espacio tridimensional cartesiano
 * referidos al origen de la base del robot.
 *
 * @param[in]  q_deg   Arreglo de 3 elementos con las coordenadas articulares [Q1, Q2, Q3] en [°].
 * @param[out] pos_mm  Arreglo de 3 elementos donde se retornan las coordenadas cartesianas [X, Y, Z] en [mm].
 *
 * @return true si el cálculo es matemáticamente admisible, false ante errores numéricos.
 */
bool Kinematics_FK(const float q_deg[3], float pos_mm[3]);

/**
 * @brief Resuelve la cinemática inversa analítica (Inverse Kinematics).
 * @details Determina los ángulos articulares requeridos a partir de una posición cartesiana objetivo,
 * forzando analíticamente la rama geométrica de configuración "codo arriba".
 *
 * @param[in]  pos_mm  Coordenadas cartesianas objetivo [X, Y, Z] en [mm].
 * @param[out] q_deg   Arreglo de salida donde se depositan los ángulos articulares [Q1, Q2, Q3] en [°].
 *
 * @return true si la pose es alcanzable dentro del espacio de trabajo y respeta los límites mecánicos;
 *         false si la posición es inalcanzable o viola las cotas de las articulaciones.
 */
bool Kinematics_IK(const float pos_mm[3], float q_deg[3]);

/**
 * @brief Evalúa si un vector de ángulos articulares cumple con los límites de carrera seguros.
 *
 * @param[in] q_deg Arreglo de 3 elementos con los ángulos articulares a evaluar [Q1, Q2, Q3] en [°].
 * @return true si los 3 ejes están dentro de los límites admisibles; false en caso contrario.
 */
bool Kinematics_CheckJointLimits(const float q_deg[3]);

#ifdef __cplusplus
}
#endif

#endif /* INC_KINEMATICS_H_ */
