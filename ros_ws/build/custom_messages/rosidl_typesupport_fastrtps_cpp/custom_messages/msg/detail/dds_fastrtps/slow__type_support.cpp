// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from custom_messages:msg/Slow.idl
// generated code does not contain a copyright notice
#include "custom_messages/msg/detail/slow__rosidl_typesupport_fastrtps_cpp.hpp"
#include "custom_messages/msg/detail/slow__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace custom_messages
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_messages
cdr_serialize(
  const custom_messages::msg::Slow & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: slowcmdvel
  cdr << ros_message.slowcmdvel;
  // Member: slowcmdang
  cdr << ros_message.slowcmdang;
  // Member: slowcmdlogi
  cdr << (ros_message.slowcmdlogi ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_messages
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  custom_messages::msg::Slow & ros_message)
{
  // Member: slowcmdvel
  cdr >> ros_message.slowcmdvel;

  // Member: slowcmdang
  cdr >> ros_message.slowcmdang;

  // Member: slowcmdlogi
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.slowcmdlogi = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_messages
get_serialized_size(
  const custom_messages::msg::Slow & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: slowcmdvel
  {
    size_t item_size = sizeof(ros_message.slowcmdvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: slowcmdang
  {
    size_t item_size = sizeof(ros_message.slowcmdang);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: slowcmdlogi
  {
    size_t item_size = sizeof(ros_message.slowcmdlogi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_messages
max_serialized_size_Slow(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: slowcmdvel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: slowcmdang
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: slowcmdlogi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom_messages::msg::Slow;
    is_plain =
      (
      offsetof(DataType, slowcmdlogi) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Slow__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const custom_messages::msg::Slow *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Slow__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<custom_messages::msg::Slow *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Slow__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const custom_messages::msg::Slow *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Slow__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Slow(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Slow__callbacks = {
  "custom_messages::msg",
  "Slow",
  _Slow__cdr_serialize,
  _Slow__cdr_deserialize,
  _Slow__get_serialized_size,
  _Slow__max_serialized_size
};

static rosidl_message_type_support_t _Slow__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Slow__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace custom_messages

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_custom_messages
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_messages::msg::Slow>()
{
  return &custom_messages::msg::typesupport_fastrtps_cpp::_Slow__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_messages, msg, Slow)() {
  return &custom_messages::msg::typesupport_fastrtps_cpp::_Slow__handle;
}

#ifdef __cplusplus
}
#endif
