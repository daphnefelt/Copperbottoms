// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/Slow.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__SLOW__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__SLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/slow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_Slow_slowcmdlogi
{
public:
  explicit Init_Slow_slowcmdlogi(::custom_messages::msg::Slow & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::Slow slowcmdlogi(::custom_messages::msg::Slow::_slowcmdlogi_type arg)
  {
    msg_.slowcmdlogi = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::Slow msg_;
};

class Init_Slow_slowcmdang
{
public:
  explicit Init_Slow_slowcmdang(::custom_messages::msg::Slow & msg)
  : msg_(msg)
  {}
  Init_Slow_slowcmdlogi slowcmdang(::custom_messages::msg::Slow::_slowcmdang_type arg)
  {
    msg_.slowcmdang = std::move(arg);
    return Init_Slow_slowcmdlogi(msg_);
  }

private:
  ::custom_messages::msg::Slow msg_;
};

class Init_Slow_slowcmdvel
{
public:
  Init_Slow_slowcmdvel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Slow_slowcmdang slowcmdvel(::custom_messages::msg::Slow::_slowcmdvel_type arg)
  {
    msg_.slowcmdvel = std::move(arg);
    return Init_Slow_slowcmdang(msg_);
  }

private:
  ::custom_messages::msg::Slow msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::Slow>()
{
  return custom_messages::msg::builder::Init_Slow_slowcmdvel();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__SLOW__BUILDER_HPP_
