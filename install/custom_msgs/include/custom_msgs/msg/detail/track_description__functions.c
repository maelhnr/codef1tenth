// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/TrackDescription.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/track_description__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `centerline`
// Member `inside_wall`
// Member `outside_wall`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
custom_msgs__msg__TrackDescription__init(custom_msgs__msg__TrackDescription * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_msgs__msg__TrackDescription__fini(msg);
    return false;
  }
  // success
  // centerline
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->centerline, 0)) {
    custom_msgs__msg__TrackDescription__fini(msg);
    return false;
  }
  // inside_wall
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->inside_wall, 0)) {
    custom_msgs__msg__TrackDescription__fini(msg);
    return false;
  }
  // outside_wall
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->outside_wall, 0)) {
    custom_msgs__msg__TrackDescription__fini(msg);
    return false;
  }
  return true;
}

void
custom_msgs__msg__TrackDescription__fini(custom_msgs__msg__TrackDescription * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // success
  // centerline
  geometry_msgs__msg__Pose__Sequence__fini(&msg->centerline);
  // inside_wall
  geometry_msgs__msg__Pose__Sequence__fini(&msg->inside_wall);
  // outside_wall
  geometry_msgs__msg__Pose__Sequence__fini(&msg->outside_wall);
}

bool
custom_msgs__msg__TrackDescription__are_equal(const custom_msgs__msg__TrackDescription * lhs, const custom_msgs__msg__TrackDescription * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // centerline
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->centerline), &(rhs->centerline)))
  {
    return false;
  }
  // inside_wall
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->inside_wall), &(rhs->inside_wall)))
  {
    return false;
  }
  // outside_wall
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->outside_wall), &(rhs->outside_wall)))
  {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__TrackDescription__copy(
  const custom_msgs__msg__TrackDescription * input,
  custom_msgs__msg__TrackDescription * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // success
  output->success = input->success;
  // centerline
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->centerline), &(output->centerline)))
  {
    return false;
  }
  // inside_wall
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->inside_wall), &(output->inside_wall)))
  {
    return false;
  }
  // outside_wall
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->outside_wall), &(output->outside_wall)))
  {
    return false;
  }
  return true;
}

custom_msgs__msg__TrackDescription *
custom_msgs__msg__TrackDescription__create()
{
  custom_msgs__msg__TrackDescription * msg = (custom_msgs__msg__TrackDescription *)malloc(sizeof(custom_msgs__msg__TrackDescription));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__TrackDescription));
  bool success = custom_msgs__msg__TrackDescription__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__TrackDescription__destroy(custom_msgs__msg__TrackDescription * msg)
{
  if (msg) {
    custom_msgs__msg__TrackDescription__fini(msg);
  }
  free(msg);
}


bool
custom_msgs__msg__TrackDescription__Sequence__init(custom_msgs__msg__TrackDescription__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  custom_msgs__msg__TrackDescription * data = NULL;
  if (size) {
    data = (custom_msgs__msg__TrackDescription *)calloc(size, sizeof(custom_msgs__msg__TrackDescription));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__TrackDescription__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__TrackDescription__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__TrackDescription__Sequence__fini(custom_msgs__msg__TrackDescription__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__TrackDescription__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__TrackDescription__Sequence *
custom_msgs__msg__TrackDescription__Sequence__create(size_t size)
{
  custom_msgs__msg__TrackDescription__Sequence * array = (custom_msgs__msg__TrackDescription__Sequence *)malloc(sizeof(custom_msgs__msg__TrackDescription__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__TrackDescription__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__TrackDescription__Sequence__destroy(custom_msgs__msg__TrackDescription__Sequence * array)
{
  if (array) {
    custom_msgs__msg__TrackDescription__Sequence__fini(array);
  }
  free(array);
}

bool
custom_msgs__msg__TrackDescription__Sequence__are_equal(const custom_msgs__msg__TrackDescription__Sequence * lhs, const custom_msgs__msg__TrackDescription__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__TrackDescription__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__TrackDescription__Sequence__copy(
  const custom_msgs__msg__TrackDescription__Sequence * input,
  custom_msgs__msg__TrackDescription__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__TrackDescription);
    custom_msgs__msg__TrackDescription * data =
      (custom_msgs__msg__TrackDescription *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__TrackDescription__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__TrackDescription__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__TrackDescription__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
