// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/Slow.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Slow in the package custom_messages.
typedef struct custom_messages__msg__Slow
{
  float slowcmdvel;
  float slowcmdang;
  bool slowcmdlogi;
} custom_messages__msg__Slow;

// Struct for a sequence of custom_messages__msg__Slow.
typedef struct custom_messages__msg__Slow__Sequence
{
  custom_messages__msg__Slow * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__Slow__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_H_
