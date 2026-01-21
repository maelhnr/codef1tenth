// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/LapProgress.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__BUILDER_HPP_

#include "custom_msgs/msg/detail/lap_progress__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_LapProgress_lap_progress
{
public:
  explicit Init_LapProgress_lap_progress(::custom_msgs::msg::LapProgress & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::LapProgress lap_progress(::custom_msgs::msg::LapProgress::_lap_progress_type arg)
  {
    msg_.lap_progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::LapProgress msg_;
};

class Init_LapProgress_lap_number
{
public:
  explicit Init_LapProgress_lap_number(::custom_msgs::msg::LapProgress & msg)
  : msg_(msg)
  {}
  Init_LapProgress_lap_progress lap_number(::custom_msgs::msg::LapProgress::_lap_number_type arg)
  {
    msg_.lap_number = std::move(arg);
    return Init_LapProgress_lap_progress(msg_);
  }

private:
  ::custom_msgs::msg::LapProgress msg_;
};

class Init_LapProgress_header
{
public:
  Init_LapProgress_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LapProgress_lap_number header(::custom_msgs::msg::LapProgress::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LapProgress_lap_number(msg_);
  }

private:
  ::custom_msgs::msg::LapProgress msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::LapProgress>()
{
  return custom_msgs::msg::builder::Init_LapProgress_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__BUILDER_HPP_
