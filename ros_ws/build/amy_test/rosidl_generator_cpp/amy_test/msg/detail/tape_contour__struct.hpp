// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_HPP_
#define AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__amy_test__msg__TapeContour __attribute__((deprecated))
#else
# define DEPRECATED__amy_test__msg__TapeContour __declspec(deprecated)
#endif

namespace amy_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TapeContour_
{
  using Type = TapeContour_<ContainerAllocator>;

  explicit TapeContour_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->found = false;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
      this->angle = 0.0f;
      this->area = 0.0f;
      this->sharp_turn = false;
    }
  }

  explicit TapeContour_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->found = false;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
      this->angle = 0.0f;
      this->area = 0.0f;
      this->sharp_turn = false;
    }
  }

  // field types and members
  using _found_type =
    bool;
  _found_type found;
  using _center_x_type =
    float;
  _center_x_type center_x;
  using _center_y_type =
    float;
  _center_y_type center_y;
  using _angle_type =
    float;
  _angle_type angle;
  using _area_type =
    float;
  _area_type area;
  using _sharp_turn_type =
    bool;
  _sharp_turn_type sharp_turn;

  // setters for named parameter idiom
  Type & set__found(
    const bool & _arg)
  {
    this->found = _arg;
    return *this;
  }
  Type & set__center_x(
    const float & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const float & _arg)
  {
    this->center_y = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__area(
    const float & _arg)
  {
    this->area = _arg;
    return *this;
  }
  Type & set__sharp_turn(
    const bool & _arg)
  {
    this->sharp_turn = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    amy_test::msg::TapeContour_<ContainerAllocator> *;
  using ConstRawPtr =
    const amy_test::msg::TapeContour_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<amy_test::msg::TapeContour_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<amy_test::msg::TapeContour_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      amy_test::msg::TapeContour_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<amy_test::msg::TapeContour_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      amy_test::msg::TapeContour_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<amy_test::msg::TapeContour_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<amy_test::msg::TapeContour_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<amy_test::msg::TapeContour_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__amy_test__msg__TapeContour
    std::shared_ptr<amy_test::msg::TapeContour_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__amy_test__msg__TapeContour
    std::shared_ptr<amy_test::msg::TapeContour_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TapeContour_ & other) const
  {
    if (this->found != other.found) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->area != other.area) {
      return false;
    }
    if (this->sharp_turn != other.sharp_turn) {
      return false;
    }
    return true;
  }
  bool operator!=(const TapeContour_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TapeContour_

// alias to use template instance with default allocator
using TapeContour =
  amy_test::msg::TapeContour_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace amy_test

#endif  // AMY_TEST__MSG__DETAIL__TAPE_CONTOUR__STRUCT_HPP_
