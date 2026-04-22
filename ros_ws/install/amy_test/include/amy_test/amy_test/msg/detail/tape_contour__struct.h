// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_H_
#define AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TapeContour in the package amy_test.
/**
  * TapeContour.msg
  * Custom message for tape contour detection
 */
typedef struct amy_test__msg__TapeContour
{
  bool found;
  float center_x;
  float center_y;
  float angle;
  float area;
  bool sharp_turn;
} amy_test__msg__TapeContour;

// Struct for a sequence of amy_test__msg__TapeContour.
typedef struct amy_test__msg__TapeContour__Sequence
{
  amy_test__msg__TapeContour * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} amy_test__msg__TapeContour__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_H_
