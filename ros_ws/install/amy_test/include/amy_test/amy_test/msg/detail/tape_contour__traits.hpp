// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__TRAITS_HPP_
#define AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "amy_test/msg/detail/tape_contour__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace amy_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const TapeContour & msg,
  std::ostream & out)
{
  out << "{";
  // member: found
  {
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << ", ";
  }

  // member: center_x
  {
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << ", ";
  }

  // member: center_y
  {
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: area
  {
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
    out << ", ";
  }

  // member: sharp_turn
  {
    out << "sharp_turn: ";
    rosidl_generator_traits::value_to_yaml(msg.sharp_turn, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TapeContour & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: found
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "found: ";
    rosidl_generator_traits::value_to_yaml(msg.found, out);
    out << "\n";
  }

  // member: center_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << "\n";
  }

  // member: center_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
    out << "\n";
  }

  // member: sharp_turn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sharp_turn: ";
    rosidl_generator_traits::value_to_yaml(msg.sharp_turn, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TapeContour & msg, bool use_flow_style = false)
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

}  // namespace amy_test

namespace rosidl_generator_traits
{

[[deprecated("use amy_test::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const amy_test::msg::TapeContour & msg,
  std::ostream & out, size_t indentation = 0)
{
  amy_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use amy_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const amy_test::msg::TapeContour & msg)
{
  return amy_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<amy_test::msg::TapeContour>()
{
  return "amy_test::msg::TapeContour";
}

template<>
inline const char * name<amy_test::msg::TapeContour>()
{
  return "amy_test/msg/TapeContour";
}

template<>
struct has_fixed_size<amy_test::msg::TapeContour>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<amy_test::msg::TapeContour>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<amy_test::msg::TapeContour>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__TRAITS_HPP_
