// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/ImuBundled.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/imu_bundled__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_ImuBundled_gyro
{
public:
  explicit Init_ImuBundled_gyro(::custom_messages::msg::ImuBundled & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::ImuBundled gyro(::custom_messages::msg::ImuBundled::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::ImuBundled msg_;
};

class Init_ImuBundled_accel
{
public:
  Init_ImuBundled_accel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuBundled_gyro accel(::custom_messages::msg::ImuBundled::_accel_type arg)
  {
    msg_.accel = std::move(arg);
    return Init_ImuBundled_gyro(msg_);
  }

private:
  ::custom_messages::msg::ImuBundled msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::ImuBundled>()
{
  return custom_messages::msg::builder::Init_ImuBundled_accel();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__IMU_BUNDLED__BUILDER_HPP_
