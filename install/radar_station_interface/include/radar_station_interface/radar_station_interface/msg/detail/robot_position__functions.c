// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice
#include "radar_station_interface/msg/detail/robot_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
radar_station_interface__msg__RobotPosition__init(radar_station_interface__msg__RobotPosition * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // width
  // height
  // depth
  // id
  return true;
}

void
radar_station_interface__msg__RobotPosition__fini(radar_station_interface__msg__RobotPosition * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // width
  // height
  // depth
  // id
}

bool
radar_station_interface__msg__RobotPosition__are_equal(const radar_station_interface__msg__RobotPosition * lhs, const radar_station_interface__msg__RobotPosition * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // depth
  if (lhs->depth != rhs->depth) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
radar_station_interface__msg__RobotPosition__copy(
  const radar_station_interface__msg__RobotPosition * input,
  radar_station_interface__msg__RobotPosition * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // depth
  output->depth = input->depth;
  // id
  output->id = input->id;
  return true;
}

radar_station_interface__msg__RobotPosition *
radar_station_interface__msg__RobotPosition__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPosition * msg = (radar_station_interface__msg__RobotPosition *)allocator.allocate(sizeof(radar_station_interface__msg__RobotPosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_station_interface__msg__RobotPosition));
  bool success = radar_station_interface__msg__RobotPosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_station_interface__msg__RobotPosition__destroy(radar_station_interface__msg__RobotPosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_station_interface__msg__RobotPosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_station_interface__msg__RobotPosition__Sequence__init(radar_station_interface__msg__RobotPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPosition * data = NULL;

  if (size) {
    data = (radar_station_interface__msg__RobotPosition *)allocator.zero_allocate(size, sizeof(radar_station_interface__msg__RobotPosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_station_interface__msg__RobotPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_station_interface__msg__RobotPosition__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
radar_station_interface__msg__RobotPosition__Sequence__fini(radar_station_interface__msg__RobotPosition__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      radar_station_interface__msg__RobotPosition__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

radar_station_interface__msg__RobotPosition__Sequence *
radar_station_interface__msg__RobotPosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPosition__Sequence * array = (radar_station_interface__msg__RobotPosition__Sequence *)allocator.allocate(sizeof(radar_station_interface__msg__RobotPosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_station_interface__msg__RobotPosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_station_interface__msg__RobotPosition__Sequence__destroy(radar_station_interface__msg__RobotPosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_station_interface__msg__RobotPosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_station_interface__msg__RobotPosition__Sequence__are_equal(const radar_station_interface__msg__RobotPosition__Sequence * lhs, const radar_station_interface__msg__RobotPosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_station_interface__msg__RobotPosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_station_interface__msg__RobotPosition__Sequence__copy(
  const radar_station_interface__msg__RobotPosition__Sequence * input,
  radar_station_interface__msg__RobotPosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_station_interface__msg__RobotPosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_station_interface__msg__RobotPosition * data =
      (radar_station_interface__msg__RobotPosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_station_interface__msg__RobotPosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_station_interface__msg__RobotPosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_station_interface__msg__RobotPosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
