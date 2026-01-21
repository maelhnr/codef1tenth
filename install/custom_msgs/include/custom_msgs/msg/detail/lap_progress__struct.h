// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/LapProgress.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_H_

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

// Struct defined in msg/LapProgress in the package custom_msgs.
typedef struct custom_msgs__msg__LapProgress
{
  std_msgs__msg__Header header;
  uint16_t lap_number;
  float lap_progress;
} custom_msgs__msg__LapProgress;

// Struct for a sequence of custom_msgs__msg__LapProgress.
typedef struct custom_msgs__msg__LapProgress__Sequence
{
  custom_msgs__msg__LapProgress * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__LapProgress__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__LAP_PROGRESS__STRUCT_H_
