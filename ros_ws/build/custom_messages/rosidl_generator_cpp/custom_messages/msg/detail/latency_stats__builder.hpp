// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/LatencyStats.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__LATENCY_STATS__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__LATENCY_STATS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/latency_stats__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::LatencyStats>()
{
  return ::custom_messages::msg::LatencyStats(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__LATENCY_STATS__BUILDER_HPP_
