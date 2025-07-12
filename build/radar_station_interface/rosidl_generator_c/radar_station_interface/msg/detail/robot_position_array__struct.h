// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_H_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'positions'
#include "radar_station_interface/msg/detail/robot_position__struct.h"

/// Struct defined in msg/RobotPositionArray in the package radar_station_interface.
/**
  * msg/RobotPositionArray
 */
typedef struct radar_station_interface__msg__RobotPositionArray
{
  radar_station_interface__msg__RobotPosition__Sequence positions;
} radar_station_interface__msg__RobotPositionArray;

// Struct for a sequence of radar_station_interface__msg__RobotPositionArray.
typedef struct radar_station_interface__msg__RobotPositionArray__Sequence
{
  radar_station_interface__msg__RobotPositionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_station_interface__msg__RobotPositionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_H_
