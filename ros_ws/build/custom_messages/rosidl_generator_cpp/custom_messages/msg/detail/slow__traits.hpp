// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_messages:msg/Slow.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__SLOW__TRAITS_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__SLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_messages/msg/detail/slow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const Slow & msg,
  std::ostream & out)
{
  out << "{";
  // member: slowcmdvel
  {
    out << "slowcmdvel: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdvel, out);
    out << ", ";
  }

  // member: slowcmdang
  {
    out << "slowcmdang: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdang, out);
    out << ", ";
  }

  // member: slowcmdlogi
  {
    out << "slowcmdlogi: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdlogi, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Slow & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: slowcmdvel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slowcmdvel: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdvel, out);
    out << "\n";
  }

  // member: slowcmdang
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slowcmdang: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdang, out);
    out << "\n";
  }

  // member: slowcmdlogi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slowcmdlogi: ";
    rosidl_generator_traits::value_to_yaml(msg.slowcmdlogi, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Slow & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_messages

namespace rosidl_generator_traits
{

[[deprecated("use custom_messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_messages::msg::Slow & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_messages::msg::Slow & msg)
{
  return custom_messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_messages::msg::Slow>()
{
  return "custom_messages::msg::Slow";
}

template<>
inline const char * name<custom_messages::msg::Slow>()
{
  return "custom_messages/msg/Slow";
}

template<>
struct has_fixed_size<custom_messages::msg::Slow>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_messages::msg::Slow>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_messages::msg::Slow>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__SLOW__TRAITS_HPP_
