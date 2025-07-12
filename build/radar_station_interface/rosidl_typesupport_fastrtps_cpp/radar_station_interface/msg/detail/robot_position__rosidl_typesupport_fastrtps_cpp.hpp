// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "radar_station_interface/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "radar_station_interface/msg/detail/robot_position__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace radar_station_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_radar_station_interface
cdr_serialize(
  const radar_station_interface::msg::RobotPosition & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_radar_station_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  radar_station_interface::msg::RobotPosition & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_radar_station_interface
get_serialized_size(
  const radar_station_interface::msg::RobotPosition & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_radar_station_interface
max_serialized_size_RobotPosition(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace radar_station_interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_radar_station_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, radar_station_interface, msg, RobotPosition)();

#ifdef __cplusplus
}
#endif

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
