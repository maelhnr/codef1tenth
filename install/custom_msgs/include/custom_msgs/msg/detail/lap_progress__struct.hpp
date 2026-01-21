// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/LapProgress.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__LapProgress __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__LapProgress __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LapProgress_
{
  using Type = LapProgress_<ContainerAllocator>;

  explicit LapProgress_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_number = 0;
      this->lap_progress = 0.0f;
    }
  }

  explicit LapProgress_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_number = 0;
      this->lap_progress = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _lap_number_type =
    uint16_t;
  _lap_number_type lap_number;
  using _lap_progress_type =
    float;
  _lap_progress_type lap_progress;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__lap_number(
    const uint16_t & _arg)
  {
    this->lap_number = _arg;
    return *this;
  }
  Type & set__lap_progress(
    const float & _arg)
  {
    this->lap_progress = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::LapProgress_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::LapProgress_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::LapProgress_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::LapProgress_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__LapProgress
    std::shared_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__LapProgress
    std::shared_ptr<custom_msgs::msg::LapProgress_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LapProgress_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->lap_number != other.lap_number) {
      return false;
    }
    if (this->lap_progress != other.lap_progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const LapProgress_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LapProgress_

// alias to use template instance with default allocator
using LapProgress =
  custom_msgs::msg::LapProgress_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_HPP_
