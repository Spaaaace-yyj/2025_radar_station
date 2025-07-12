// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from livox_interfaces:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_INTERFACES__MSG__DETAIL__CUSTOM_POINT__STRUCT_H_
#define LIVOX_INTERFACES__MSG__DETAIL__CUSTOM_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CustomPoint in the package livox_interfaces.
/**
  * Livox costom pointcloud format.
 */
typedef struct livox_interfaces__msg__CustomPoint
{
  /// offset time relative to the base time
  uint32_t offset_time;
  /// X axis, unit:m
  float x;
  /// Y axis, unit:m
  float y;
  /// Z axis, unit:m
  float z;
  /// reflectivity, 0~255
  uint8_t reflectivity;
  /// livox tag
  uint8_t tag;
  /// laser number in lidar
  uint8_t line;
} livox_interfaces__msg__CustomPoint;

// Struct for a sequence of livox_interfaces__msg__CustomPoint.
typedef struct livox_interfaces__msg__CustomPoint__Sequence
{
  livox_interfaces__msg__CustomPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} livox_interfaces__msg__CustomPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_INTERFACES__MSG__DETAIL__CUSTOM_POINT__STRUCT_H_
