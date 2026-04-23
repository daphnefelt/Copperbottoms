// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__BUILDER_HPP_
#define AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "amy_test/msg/detail/tape_contour__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace amy_test
{

namespace msg
{

namespace builder
{

class Init_TapeContour_sharp_turn
{
public:
  explicit Init_TapeContour_sharp_turn(::amy_test::msg::TapeContour & msg)
  : msg_(msg)
  {}
  ::amy_test::msg::TapeContour sharp_turn(::amy_test::msg::TapeContour::_sharp_turn_type arg)
  {
    msg_.sharp_turn = std::move(arg);
    return std::move(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

class Init_TapeContour_area
{
public:
  explicit Init_TapeContour_area(::amy_test::msg::TapeContour & msg)
  : msg_(msg)
  {}
  Init_TapeContour_sharp_turn area(::amy_test::msg::TapeContour::_area_type arg)
  {
    msg_.area = std::move(arg);
    return Init_TapeContour_sharp_turn(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

class Init_TapeContour_angle
{
public:
  explicit Init_TapeContour_angle(::amy_test::msg::TapeContour & msg)
  : msg_(msg)
  {}
  Init_TapeContour_area angle(::amy_test::msg::TapeContour::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_TapeContour_area(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

class Init_TapeContour_center_y
{
public:
  explicit Init_TapeContour_center_y(::amy_test::msg::TapeContour & msg)
  : msg_(msg)
  {}
  Init_TapeContour_angle center_y(::amy_test::msg::TapeContour::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return Init_TapeContour_angle(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

class Init_TapeContour_center_x
{
public:
  explicit Init_TapeContour_center_x(::amy_test::msg::TapeContour & msg)
  : msg_(msg)
  {}
  Init_TapeContour_center_y center_x(::amy_test::msg::TapeContour::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_TapeContour_center_y(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

class Init_TapeContour_found
{
public:
  Init_TapeContour_found()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TapeContour_center_x found(::amy_test::msg::TapeContour::_found_type arg)
  {
    msg_.found = std::move(arg);
    return Init_TapeContour_center_x(msg_);
  }

private:
  ::amy_test::msg::TapeContour msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::amy_test::msg::TapeContour>()
{
  return amy_test::msg::builder::Init_TapeContour_found();
}

}  // namespace amy_test

#endif  // AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__BUILDER_HPP_
