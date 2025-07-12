// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice
#include "radar_station_interface/msg/detail/robot_position_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `positions`
#include "radar_station_interface/msg/detail/robot_position__functions.h"

bool
radar_station_interface__msg__RobotPositionArray__init(radar_station_interface__msg__RobotPositionArray * msg)
{
  if (!msg) {
    return false;
  }
  // positions
  if (!radar_station_interface__msg__RobotPosition__Sequence__init(&msg->positions, 0)) {
    radar_station_interface__msg__RobotPositionArray__fini(msg);
    return false;
  }
  return true;
}

void
radar_station_interface__msg__RobotPositionArray__fini(radar_station_interface__msg__RobotPositionArray * msg)
{
  if (!msg) {
    return;
  }
  // positions
  radar_station_interface__msg__RobotPosition__Sequence__fini(&msg->positions);
}

bool
radar_station_interface__msg__RobotPositionArray__are_equal(const radar_station_interface__msg__RobotPositionArray * lhs, const radar_station_interface__msg__RobotPositionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // positions
  if (!radar_station_interface__msg__RobotPosition__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  return true;
}

bool
radar_station_interface__msg__RobotPositionArray__copy(
  const radar_station_interface__msg__RobotPositionArray * input,
  radar_station_interface__msg__RobotPositionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // positions
  if (!radar_station_interface__msg__RobotPosition__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  return true;
}

radar_station_interface__msg__RobotPositionArray *
radar_station_interface__msg__RobotPositionArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPositionArray * msg = (radar_station_interface__msg__RobotPositionArray *)allocator.allocate(sizeof(radar_station_interface__msg__RobotPositionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_station_interface__msg__RobotPositionArray));
  bool success = radar_station_interface__msg__RobotPositionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_station_interface__msg__RobotPositionArray__destroy(radar_station_interface__msg__RobotPositionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_station_interface__msg__RobotPositionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_station_interface__msg__RobotPositionArray__Sequence__init(radar_station_interface__msg__RobotPositionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPositionArray * data = NULL;

  if (size) {
    data = (radar_station_interface__msg__RobotPositionArray *)allocator.zero_allocate(size, sizeof(radar_station_interface__msg__RobotPositionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_station_interface__msg__RobotPositionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_station_interface__msg__RobotPositionArray__fini(&data[i - 1]);
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
radar_station_interface__msg__RobotPositionArray__Sequence__fini(radar_station_interface__msg__RobotPositionArray__Sequence * array)
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
      radar_station_interface__msg__RobotPositionArray__fini(&array->data[i]);
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

radar_station_interface__msg__RobotPositionArray__Sequence *
radar_station_interface__msg__RobotPositionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_station_interface__msg__RobotPositionArray__Sequence * array = (radar_station_interface__msg__RobotPositionArray__Sequence *)allocator.allocate(sizeof(radar_station_interface__msg__RobotPositionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_station_interface__msg__RobotPositionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_station_interface__msg__RobotPositionArray__Sequence__destroy(radar_station_interface__msg__RobotPositionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_station_interface__msg__RobotPositionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_station_interface__msg__RobotPositionArray__Sequence__are_equal(const radar_station_interface__msg__RobotPositionArray__Sequence * lhs, const radar_station_interface__msg__RobotPositionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_station_interface__msg__RobotPositionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_station_interface__msg__RobotPositionArray__Sequence__copy(
  const radar_station_interface__msg__RobotPositionArray__Sequence * input,
  radar_station_interface__msg__RobotPositionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_station_interface__msg__RobotPositionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_station_interface__msg__RobotPositionArray * data =
      (radar_station_interface__msg__RobotPositionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_station_interface__msg__RobotPositionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_station_interface__msg__RobotPositionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_station_interface__msg__RobotPositionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
