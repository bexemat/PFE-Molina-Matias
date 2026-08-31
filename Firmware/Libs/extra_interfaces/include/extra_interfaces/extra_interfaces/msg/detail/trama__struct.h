// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice

#ifndef EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_
#define EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Trama in the package extra_interfaces.
typedef struct extra_interfaces__msg__Trama
{
  /// float64[3] q - Joint positions
  double q[3];
  /// float64[3] qd - Joint velocities
  double qd[3];
  /// float64 t_total - Total time
  double t_total;
  /// int32 n_iter - Number of iterations
  int32_t n_iter;
  int32_t traj_state;
} extra_interfaces__msg__Trama;

// Struct for a sequence of extra_interfaces__msg__Trama.
typedef struct extra_interfaces__msg__Trama__Sequence
{
  extra_interfaces__msg__Trama * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} extra_interfaces__msg__Trama__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXTRA_INTERFACES__MSG__DETAIL__TRAMA__STRUCT_H_
