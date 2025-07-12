// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_station_interface/msg/detail/robot_position_array__rosidl_typesupport_introspection_c.h"
#include "radar_station_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_station_interface/msg/detail/robot_position_array__functions.h"
#include "radar_station_interface/msg/detail/robot_position_array__struct.h"


// Include directives for member types
// Member `positions`
#include "radar_station_interface/msg/robot_position.h"
// Member `positions`
#include "radar_station_interface/msg/detail/robot_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_station_interface__msg__RobotPositionArray__init(message_memory);
}

void radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_fini_function(void * message_memory)
{
  radar_station_interface__msg__RobotPositionArray__fini(message_memory);
}

size_t radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__size_function__RobotPositionArray__positions(
  const void * untyped_member)
{
  const radar_station_interface__msg__RobotPosition__Sequence * member =
    (const radar_station_interface__msg__RobotPosition__Sequence *)(untyped_member);
  return member->size;
}

const void * radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_const_function__RobotPositionArray__positions(
  const void * untyped_member, size_t index)
{
  const radar_station_interface__msg__RobotPosition__Sequence * member =
    (const radar_station_interface__msg__RobotPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void * radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_function__RobotPositionArray__positions(
  void * untyped_member, size_t index)
{
  radar_station_interface__msg__RobotPosition__Sequence * member =
    (radar_station_interface__msg__RobotPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__fetch_function__RobotPositionArray__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const radar_station_interface__msg__RobotPosition * item =
    ((const radar_station_interface__msg__RobotPosition *)
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_const_function__RobotPositionArray__positions(untyped_member, index));
  radar_station_interface__msg__RobotPosition * value =
    (radar_station_interface__msg__RobotPosition *)(untyped_value);
  *value = *item;
}

void radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__assign_function__RobotPositionArray__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  radar_station_interface__msg__RobotPosition * item =
    ((radar_station_interface__msg__RobotPosition *)
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_function__RobotPositionArray__positions(untyped_member, index));
  const radar_station_interface__msg__RobotPosition * value =
    (const radar_station_interface__msg__RobotPosition *)(untyped_value);
  *item = *value;
}

bool radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__resize_function__RobotPositionArray__positions(
  void * untyped_member, size_t size)
{
  radar_station_interface__msg__RobotPosition__Sequence * member =
    (radar_station_interface__msg__RobotPosition__Sequence *)(untyped_member);
  radar_station_interface__msg__RobotPosition__Sequence__fini(member);
  return radar_station_interface__msg__RobotPosition__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_member_array[1] = {
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface__msg__RobotPositionArray, positions),  // bytes offset in struct
    NULL,  // default value
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__size_function__RobotPositionArray__positions,  // size() function pointer
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_const_function__RobotPositionArray__positions,  // get_const(index) function pointer
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__get_function__RobotPositionArray__positions,  // get(index) function pointer
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__fetch_function__RobotPositionArray__positions,  // fetch(index, &value) function pointer
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__assign_function__RobotPositionArray__positions,  // assign(index, value) function pointer
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__resize_function__RobotPositionArray__positions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_members = {
  "radar_station_interface__msg",  // message namespace
  "RobotPositionArray",  // message name
  1,  // number of fields
  sizeof(radar_station_interface__msg__RobotPositionArray),
  radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_member_array,  // message members
  radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_type_support_handle = {
  0,
  &radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_station_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_station_interface, msg, RobotPositionArray)() {
  radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_station_interface, msg, RobotPosition)();
  if (!radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_type_support_handle.typesupport_identifier) {
    radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_station_interface__msg__RobotPositionArray__rosidl_typesupport_introspection_c__RobotPositionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
