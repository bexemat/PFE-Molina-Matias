/* Inc/kinematics.h */
#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include <stdbool.h>

/* =========================================================================
 * PARÁMETROS GEOMÉTRICOS Y DIMENSIONES VALIDADOS (en metros)
 * ========================================================================= */
#define KIN_L1          0.07700f    // Altura base al hombro (77.00 mm)
#define KIN_L2          0.13500f    // Eslabón brazo principal (135.00 mm)
#define KIN_L3          0.14700f    // Eslabón antebrazo (147.00 mm)
#define KIN_O_R         0.02549f    // Offset radial del efector (25.49 mm)
#define KIN_O_Z         0.04087f    // Offset vertical del efector (40.87 mm)

#define RAD_TO_DEG      (180.0f / 3.14159265358979323846f)
#define DEG_TO_RAD_F    (3.14159265358979323846f / 180.0f)

/* Límites articulares reales del manipulador */
#define Q1_MIN_DEG     -90.0f
#define Q1_MAX_DEG      90.0f
#define Q2_MIN_DEG      40.0f
#define Q2_MAX_DEG     157.0f
#define Q3_MIN_DEG     -75.0f
#define Q3_MAX_DEG      25.0f

bool Kinematics_FK(const float q_deg[3], float pos_mm[3]);
bool Kinematics_IK(const float pos_mm[3], float q_deg[3]);
bool Kinematics_CheckJointLimits(const float q_deg[3]);

#endif /* INC_KINEMATICS_H_ */
