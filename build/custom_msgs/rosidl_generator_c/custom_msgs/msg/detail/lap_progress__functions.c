// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/LapProgress.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/lap_progress__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
custom_msgs__msg__LapProgress__init(custom_msgs__msg__LapProgress * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_msgs__msg__LapProgress__fini(msg);
    return false;
  }
  // lap_number
  // lap_progress
  return true;
}

void
custom_msgs__msg__LapProgress__fini(custom_msgs__msg__LapProgress * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // lap_number
  // lap_progress
}

bool
custom_msgs__msg__LapProgress__are_equal(const custom_msgs__msg__LapProgress * lhs, const custom_msgs__msg__LapProgress * rhs)
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
  // lap_number
  if (lhs->lap_number != rhs->lap_number) {
    return false;
  }
  // lap_progress
  if (lhs->lap_progress != rhs->lap_progress) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__LapProgress__copy(
  const custom_msgs__msg__LapProgress * input,
  custom_msgs__msg__LapProgress * output)
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
  // lap_number
  output->lap_number = input->lap_number;
  // lap_progress
  output->lap_progress = input->lap_progress;
  return true;
}

custom_msgs__msg__LapProgress *
custom_msgs__msg__LapProgress__create()
{
  custom_msgs__msg__LapProgress * msg = (custom_msgs__msg__LapProgress *)malloc(sizeof(custom_msgs__msg__LapProgress));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__LapProgress));
  bool success = custom_msgs__msg__LapProgress__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__LapProgress__destroy(custom_msgs__msg__LapProgress * msg)
{
  if (msg) {
    custom_msgs__msg__LapProgress__fini(msg);
  }
  free(msg);
}


bool
custom_msgs__msg__LapProgress__Sequence__init(custom_msgs__msg__LapProgress__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  custom_msgs__msg__LapProgress * data = NULL;
  if (size) {
    data = (custom_msgs__msg__LapProgress *)calloc(size, sizeof(custom_msgs__msg__LapProgress));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__LapProgress__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__LapProgress__fini(&data[i - 1]);
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
custom_msgs__msg__LapProgress__Sequence__fini(custom_msgs__msg__LapProgress__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__LapProgress__fini(&array->data[i]);
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

custom_msgs__msg__LapProgress__Sequence *
custom_msgs__msg__LapProgress__Sequence__create(size_t size)
{
  custom_msgs__msg__LapProgress__Sequence * array = (custom_msgs__msg__LapProgress__Sequence *)malloc(sizeof(custom_msgs__msg__LapProgress__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__LapProgress__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__LapProgress__Sequence__destroy(custom_msgs__msg__LapProgress__Sequence * array)
{
  if (array) {
    custom_msgs__msg__LapProgress__Sequence__fini(array);
  }
  free(array);
}

bool
custom_msgs__msg__LapProgress__Sequence__are_equal(const custom_msgs__msg__LapProgress__Sequence * lhs, const custom_msgs__msg__LapProgress__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__LapProgress__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__LapProgress__Sequence__copy(
  const custom_msgs__msg__LapProgress__Sequence * input,
  custom_msgs__msg__LapProgress__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__LapProgress);
    custom_msgs__msg__LapProgress * data =
      (custom_msgs__msg__LapProgress *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__LapProgress__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__LapProgress__fini(&data[i]);
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
    if (!custom_msgs__msg__LapProgress__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
