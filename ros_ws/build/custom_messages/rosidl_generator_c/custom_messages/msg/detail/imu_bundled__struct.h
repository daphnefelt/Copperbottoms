// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/ImuBundled.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'accel'
// Member 'gyro'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ImuBundled in the package custom_messages.
typedef struct custom_messages__msg__ImuBundled
{
  geometry_msgs__msg__Vector3 accel;
  geometry_msgs__msg__Vector3 gyro;
} custom_messages__msg__ImuBundled;

// Struct for a sequence of custom_messages__msg__ImuBundled.
typedef struct custom_messages__msg__ImuBundled__Sequence
{
  custom_messages__msg__ImuBundled * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__ImuBundled__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_H_
