// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__BUILDER_HPP_

#include "custom_msgs/msg/detail/track_description__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_TrackDescription_outside_wall
{
public:
  explicit Init_TrackDescription_outside_wall(::custom_msgs::msg::TrackDescription & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::TrackDescription outside_wall(::custom_msgs::msg::TrackDescription::_outside_wall_type arg)
  {
    msg_.outside_wall = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::TrackDescription msg_;
};

class Init_TrackDescription_inside_wall
{
public:
  explicit Init_TrackDescription_inside_wall(::custom_msgs::msg::TrackDescription & msg)
  : msg_(msg)
  {}
  Init_TrackDescription_outside_wall inside_wall(::custom_msgs::msg::TrackDescription::_inside_wall_type arg)
  {
    msg_.inside_wall = std::move(arg);
    return Init_TrackDescription_outside_wall(msg_);
  }

private:
  ::custom_msgs::msg::TrackDescription msg_;
};

class Init_TrackDescription_centerline
{
public:
  explicit Init_TrackDescription_centerline(::custom_msgs::msg::TrackDescription & msg)
  : msg_(msg)
  {}
  Init_TrackDescription_inside_wall centerline(::custom_msgs::msg::TrackDescription::_centerline_type arg)
  {
    msg_.centerline = std::move(arg);
    return Init_TrackDescription_inside_wall(msg_);
  }

private:
  ::custom_msgs::msg::TrackDescription msg_;
};

class Init_TrackDescription_success
{
public:
  explicit Init_TrackDescription_success(::custom_msgs::msg::TrackDescription & msg)
  : msg_(msg)
  {}
  Init_TrackDescription_centerline success(::custom_msgs::msg::TrackDescription::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TrackDescription_centerline(msg_);
  }

private:
  ::custom_msgs::msg::TrackDescription msg_;
};

class Init_TrackDescription_header
{
public:
  Init_TrackDescription_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackDescription_success header(::custom_msgs::msg::TrackDescription::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackDescription_success(msg_);
  }

private:
  ::custom_msgs::msg::TrackDescription msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::TrackDescription>()
{
  return custom_msgs::msg::builder::Init_TrackDescription_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__BUILDER_HPP_
