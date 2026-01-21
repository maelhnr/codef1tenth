// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/LapProgress.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__TRAITS_HPP_

#include "custom_msgs/msg/detail/lap_progress__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const custom_msgs::msg::LapProgress & msg,
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

  // member: lap_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lap_number: ";
    value_to_yaml(msg.lap_number, out);
    out << "\n";
  }

  // member: lap_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lap_progress: ";
    value_to_yaml(msg.lap_progress, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const custom_msgs::msg::LapProgress & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<custom_msgs::msg::LapProgress>()
{
  return "custom_msgs::msg::LapProgress";
}

template<>
inline const char * name<custom_msgs::msg::LapProgress>()
{
  return "custom_msgs/msg/LapProgress";
}

template<>
struct has_fixed_size<custom_msgs::msg::LapProgress>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::LapProgress>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<custom_msgs::msg::LapProgress>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__TRAITS_HPP_
