// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'centerline'
// Member 'inside_wall'
// Member 'outside_wall'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in msg/TrackDescription in the package custom_msgs.
typedef struct custom_msgs__msg__TrackDescription
{
  std_msgs__msg__Header header;
  bool success;
  geometry_msgs__msg__Pose__Sequence centerline;
  geometry_msgs__msg__Pose__Sequence inside_wall;
  geometry_msgs__msg__Pose__Sequence outside_wall;
} custom_msgs__msg__TrackDescription;

// Struct for a sequence of custom_msgs__msg__TrackDescription.
typedef struct custom_msgs__msg__TrackDescription__Sequence
{
  custom_msgs__msg__TrackDescription * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__TrackDescription__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__TRACK_DESCRIPTION__STRUCT_H_
