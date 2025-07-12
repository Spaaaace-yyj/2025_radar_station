// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__BUILDER_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_station_interface/msg/detail/robot_position_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_station_interface
{

namespace msg
{

namespace builder
{

class Init_RobotPositionArray_positions
{
public:
  Init_RobotPositionArray_positions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_station_interface::msg::RobotPositionArray positions(::radar_station_interface::msg::RobotPositionArray::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPositionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_station_interface::msg::RobotPositionArray>()
{
  return radar_station_interface::msg::builder::Init_RobotPositionArray_positions();
}

}  // namespace radar_station_interface

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__BUILDER_HPP_
