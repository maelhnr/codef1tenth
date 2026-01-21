// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__TRAITS_HPP_

#include "custom_msgs/msg/detail/track_description__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'centerline'
// Member 'inside_wall'
// Member 'outside_wall'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const custom_msgs::msg::TrackDescription & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: centerline
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.centerline.size() == 0) {
      out << "centerline: []\n";
    } else {
      out << "centerline:\n";
      for (auto item : msg.centerline) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: inside_wall
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.inside_wall.size() == 0) {
      out << "inside_wall: []\n";
    } else {
      out << "inside_wall:\n";
      for (auto item : msg.inside_wall) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: outside_wall
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.outside_wall.size() == 0) {
      out << "outside_wall: []\n";
    } else {
      out << "outside_wall:\n";
      for (auto item : msg.outside_wall) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const custom_msgs::msg::TrackDescription & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<custom_msgs::msg::TrackDescription>()
{
  return "custom_msgs::msg::TrackDescription";
}

template<>
inline const char * name<custom_msgs::msg::TrackDescription>()
{
  return "custom_msgs/msg/TrackDescription";
}

template<>
struct has_fixed_size<custom_msgs::msg::TrackDescription>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::TrackDescription>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::TrackDescription>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__TRAITS_HPP_
