// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_HPP_

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
// Member 'centerline'
// Member 'inside_wall'
// Member 'outside_wall'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__TrackDescription __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__TrackDescription __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackDescription_
{
  using Type = TrackDescription_<ContainerAllocator>;

  explicit TrackDescription_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit TrackDescription_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _success_type =
    bool;
  _success_type success;
  using _centerline_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _centerline_type centerline;
  using _inside_wall_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _inside_wall_type inside_wall;
  using _outside_wall_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _outside_wall_type outside_wall;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__centerline(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->centerline = _arg;
    return *this;
  }
  Type & set__inside_wall(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->inside_wall = _arg;
    return *this;
  }
  Type & set__outside_wall(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->outside_wall = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::TrackDescription_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::TrackDescription_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TrackDescription_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TrackDescription_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__TrackDescription
    std::shared_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__TrackDescription
    std::shared_ptr<custom_msgs::msg::TrackDescription_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackDescription_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->centerline != other.centerline) {
      return false;
    }
    if (this->inside_wall != other.inside_wall) {
      return false;
    }
    if (this->outside_wall != other.outside_wall) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackDescription_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackDescription_

// alias to use template instance with default allocator
using TrackDescription =
  custom_msgs::msg::TrackDescription_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_HPP_
