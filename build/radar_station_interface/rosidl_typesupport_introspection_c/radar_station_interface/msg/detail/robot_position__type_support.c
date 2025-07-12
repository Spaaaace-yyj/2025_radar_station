// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_station_interface/msg/detail/robot_position__rosidl_typesupport_introspection_c.h"
#include "radar_station_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_station_interface/msg/detail/robot_position__functions.h"
#include "radar_station_interface/msg/detail/robot_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_station_interface__msg__RobotPosition__init(message_memory);
}

void radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_fini_function(void * message_memory)
{
  radar_station_interface__msg__RobotPosition__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_member_array[7] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPosition, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_members = {
  "radar_station_interface__msg",  // message namespace
  "RobotPosition",  // message name
  7,  // number of fields
  sizeof(radar_station_interface__msg__RobotPosition),
  radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_member_array,  // message members
  radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_type_support_handle = {
  0,
  &radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_station_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_station_interface, msg, RobotPosition)() {
  if (!radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_type_support_handle.typesupport_identifier) {
    radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_station_interface__msg__RobotPosition__rosidl_typesupport_introspection_c__RobotPosition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
