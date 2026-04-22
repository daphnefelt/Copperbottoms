// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "amy_test/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "amy_test/msg/detail/tape_contour__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace amy_test
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amy_test
cdr_serialize(
  const amy_test::msg::TapeContour & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amy_test
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  amy_test::msg::TapeContour & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amy_test
get_serialized_size(
  const amy_test::msg::TapeContour & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amy_test
max_serialized_size_TapeContour(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace amy_test

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amy_test
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, amy_test, msg, TapeContour)();

#ifdef __cplusplus
}
#endif

#endif  // AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
