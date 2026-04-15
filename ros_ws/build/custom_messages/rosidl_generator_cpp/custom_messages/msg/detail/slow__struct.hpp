// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/Slow.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__Slow __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__Slow __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Slow_
{
  using Type = Slow_<ContainerAllocator>;

  explicit Slow_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slowcmdvel = 0.0f;
      this->slowcmdang = 0.0f;
      this->slowcmdlogi = false;
    }
  }

  explicit Slow_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slowcmdvel = 0.0f;
      this->slowcmdang = 0.0f;
      this->slowcmdlogi = false;
    }
  }

  // field types and members
  using _slowcmdvel_type =
    float;
  _slowcmdvel_type slowcmdvel;
  using _slowcmdang_type =
    float;
  _slowcmdang_type slowcmdang;
  using _slowcmdlogi_type =
    bool;
  _slowcmdlogi_type slowcmdlogi;

  // setters for named parameter idiom
  Type & set__slowcmdvel(
    const float & _arg)
  {
    this->slowcmdvel = _arg;
    return *this;
  }
  Type & set__slowcmdang(
    const float & _arg)
  {
    this->slowcmdang = _arg;
    return *this;
  }
  Type & set__slowcmdlogi(
    const bool & _arg)
  {
    this->slowcmdlogi = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::Slow_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::Slow_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::Slow_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::Slow_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Slow_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Slow_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Slow_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Slow_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::Slow_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::Slow_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__Slow
    std::shared_ptr<custom_messages::msg::Slow_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__Slow
    std::shared_ptr<custom_messages::msg::Slow_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Slow_ & other) const
  {
    if (this->slowcmdvel != other.slowcmdvel) {
      return false;
    }
    if (this->slowcmdang != other.slowcmdang) {
      return false;
    }
    if (this->slowcmdlogi != other.slowcmdlogi) {
      return false;
    }
    return true;
  }
  bool operator!=(const Slow_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Slow_

// alias to use template instance with default allocator
using Slow =
  custom_messages::msg::Slow_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__SLOW__STRUCT_HPP_
