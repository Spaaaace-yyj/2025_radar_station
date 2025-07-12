// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "radar_station_interface/msg/detail/robot_position_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace radar_station_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RobotPositionArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) radar_station_interface::msg::RobotPositionArray(_init);
}

void RobotPositionArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<radar_station_interface::msg::RobotPositionArray *>(message_memory);
  typed_message->~RobotPositionArray();
}

size_t size_function__RobotPositionArray__positions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<radar_station_interface::msg::RobotPosition> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPositionArray__positions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<radar_station_interface::msg::RobotPosition> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPositionArray__positions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<radar_station_interface::msg::RobotPosition> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPositionArray__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const radar_station_interface::msg::RobotPosition *>(
    get_const_function__RobotPositionArray__positions(untyped_member, index));
  auto & value = *reinterpret_cast<radar_station_interface::msg::RobotPosition *>(untyped_value);
  value = item;
}

void assign_function__RobotPositionArray__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<radar_station_interface::msg::RobotPosition *>(
    get_function__RobotPositionArray__positions(untyped_member, index));
  const auto & value = *reinterpret_cast<const radar_station_interface::msg::RobotPosition *>(untyped_value);
  item = value;
}

void resize_function__RobotPositionArray__positions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<radar_station_interface::msg::RobotPosition> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotPositionArray_message_member_array[1] = {
  {
    "positions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<radar_station_interface::msg::RobotPosition>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_station_interface::msg::RobotPositionArray, positions),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPositionArray__positions,  // size() function pointer
    get_const_function__RobotPositionArray__positions,  // get_const(index) function pointer
    get_function__RobotPositionArray__positions,  // get(index) function pointer
    fetch_function__RobotPositionArray__positions,  // fetch(index, &value) function pointer
    assign_function__RobotPositionArray__positions,  // assign(index, value) function pointer
    resize_function__RobotPositionArray__positions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotPositionArray_message_members = {
  "radar_station_interface::msg",  // message namespace
  "RobotPositionArray",  // message name
  1,  // number of fields
  sizeof(radar_station_interface::msg::RobotPositionArray),
  RobotPositionArray_message_member_array,  // message members
  RobotPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotPositionArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotPositionArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace radar_station_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_station_interface::msg::RobotPositionArray>()
{
  return &::radar_station_interface::msg::rosidl_typesupport_introspection_cpp::RobotPositionArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_station_interface, msg, RobotPositionArray)() {
  return &::radar_station_interface::msg::rosidl_typesupport_introspection_cpp::RobotPositionArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
