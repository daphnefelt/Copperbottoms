// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "amy_test/msg/detail/tape_contour__rosidl_typesupport_introspection_c.h"
#include "amy_test/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "amy_test/msg/detail/tape_contour__functions.h"
#include "amy_test/msg/detail/tape_contour__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  amy_test__msg__TapeContour__init(message_memory);
}

void amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_fini_function(void * message_memory)
{
  amy_test__msg__TapeContour__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_member_array[6] = {
  {
    "found",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, found),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, center_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, center_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "area",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, area),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sharp_turn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amy_test__msg__TapeContour, sharp_turn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_members = {
  "amy_test__msg",  // message namespace
  "TapeContour",  // message name
  6,  // number of fields
  sizeof(amy_test__msg__TapeContour),
  amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_member_array,  // message members
  amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_init_function,  // function to initialize message memory (memory has to be allocated)
  amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_type_support_handle = {
  0,
  &amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_amy_test
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, amy_test, msg, TapeContour)() {
  if (!amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_type_support_handle.typesupport_identifier) {
    amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &amy_test__msg__TapeContour__rosidl_typesupport_introspection_c__TapeContour_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
