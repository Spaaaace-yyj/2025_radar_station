// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_H_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RobotPosition in the package radar_station_interface.
/**
  * msg/RobotPosition
 */
typedef struct radar_station_interface__msg__RobotPosition
{
  double x;
  double y;
  double z;
  double width;
  double height;
  double depth;
  int32_t id;
} radar_station_interface__msg__RobotPosition;

// Struct for a sequence of radar_station_interface__msg__RobotPosition.
typedef struct radar_station_interface__msg__RobotPosition__Sequence
{
  radar_station_interface__msg__RobotPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_station_interface__msg__RobotPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_H_
