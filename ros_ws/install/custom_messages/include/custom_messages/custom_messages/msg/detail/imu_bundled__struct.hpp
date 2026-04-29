// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/ImuBundled.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'accel'
// Member 'gyro'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__ImuBundled __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__ImuBundled __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuBundled_
{
  using Type = ImuBundled_<ContainerAllocator>;

  explicit ImuBundled_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_init),
    gyro(_init)
  {
    (void)_init;
  }

  explicit ImuBundled_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_alloc, _init),
    gyro(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _accel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _accel_type accel;
  using _gyro_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gyro_type gyro;

  // setters for named parameter idiom
  Type & set__accel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }
  Type & set__gyro(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gyro = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::ImuBundled_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::ImuBundled_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::ImuBundled_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::ImuBundled_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__ImuBundled
    std::shared_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__ImuBundled
    std::shared_ptr<custom_messages::msg::ImuBundled_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuBundled_ & other) const
  {
    if (this->accel != other.accel) {
      return false;
    }
    if (this->gyro != other.gyro) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuBundled_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuBundled_

// alias to use template instance with default allocator
using ImuBundled =
  custom_messages::msg::ImuBundled_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__STRUCT_HPP_
