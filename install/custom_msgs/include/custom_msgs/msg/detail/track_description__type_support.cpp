// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_msgs/msg/detail/track_description__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace custom_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TrackDescription_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) custom_msgs::msg::TrackDescription(_init);
}

void TrackDescription_fini_function(void * message_memory)
{
  auto typed_message = static_cast<custom_msgs::msg::TrackDescription *>(message_memory);
  typed_message->~TrackDescription();
}

size_t size_function__TrackDescription__centerline(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrackDescription__centerline(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__TrackDescription__centerline(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void resize_function__TrackDescription__centerline(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TrackDescription__inside_wall(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrackDescription__inside_wall(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__TrackDescription__inside_wall(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void resize_function__TrackDescription__inside_wall(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TrackDescription__outside_wall(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrackDescription__outside_wall(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__TrackDescription__outside_wall(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void resize_function__TrackDescription__outside_wall(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrackDescription_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::TrackDescription, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::TrackDescription, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "centerline",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::TrackDescription, centerline),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrackDescription__centerline,  // size() function pointer
    get_const_function__TrackDescription__centerline,  // get_const(index) function pointer
    get_function__TrackDescription__centerline,  // get(index) function pointer
    resize_function__TrackDescription__centerline  // resize(index) function pointer
  },
  {
    "inside_wall",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::TrackDescription, inside_wall),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrackDescription__inside_wall,  // size() function pointer
    get_const_function__TrackDescription__inside_wall,  // get_const(index) function pointer
    get_function__TrackDescription__inside_wall,  // get(index) function pointer
    resize_function__TrackDescription__inside_wall  // resize(index) function pointer
  },
  {
    "outside_wall",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::TrackDescription, outside_wall),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrackDescription__outside_wall,  // size() function pointer
    get_const_function__TrackDescription__outside_wall,  // get_const(index) function pointer
    get_function__TrackDescription__outside_wall,  // get(index) function pointer
    resize_function__TrackDescription__outside_wall  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrackDescription_message_members = {
  "custom_msgs::msg",  // message namespace
  "TrackDescription",  // message name
  5,  // number of fields
  sizeof(custom_msgs::msg::TrackDescription),
  TrackDescription_message_member_array,  // message members
  TrackDescription_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackDescription_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrackDescription_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrackDescription_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace custom_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_msgs::msg::TrackDescription>()
{
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::TrackDescription_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_msgs, msg, TrackDescription)() {
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::TrackDescription_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
