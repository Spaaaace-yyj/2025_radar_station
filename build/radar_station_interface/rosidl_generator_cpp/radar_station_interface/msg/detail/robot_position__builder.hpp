// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__BUILDER_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_station_interface/msg/detail/robot_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_station_interface
{

namespace msg
{

namespace builder
{

class Init_RobotPosition_id
{
public:
  explicit Init_RobotPosition_id(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  ::radar_station_interface::msg::RobotPosition id(::radar_station_interface::msg::RobotPosition::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_depth
{
public:
  explicit Init_RobotPosition_depth(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  Init_RobotPosition_id depth(::radar_station_interface::msg::RobotPosition::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return Init_RobotPosition_id(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_height
{
public:
  explicit Init_RobotPosition_height(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  Init_RobotPosition_depth height(::radar_station_interface::msg::RobotPosition::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_RobotPosition_depth(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_width
{
public:
  explicit Init_RobotPosition_width(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  Init_RobotPosition_height width(::radar_station_interface::msg::RobotPosition::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_RobotPosition_height(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_z
{
public:
  explicit Init_RobotPosition_z(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  Init_RobotPosition_width z(::radar_station_interface::msg::RobotPosition::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_RobotPosition_width(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_y
{
public:
  explicit Init_RobotPosition_y(::radar_station_interface::msg::RobotPosition & msg)
  : msg_(msg)
  {}
  Init_RobotPosition_z y(::radar_station_interface::msg::RobotPosition::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotPosition_z(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

class Init_RobotPosition_x
{
public:
  Init_RobotPosition_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPosition_y x(::radar_station_interface::msg::RobotPosition::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotPosition_y(msg_);
  }

private:
  ::radar_station_interface::msg::RobotPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_station_interface::msg::RobotPosition>()
{
  return radar_station_interface::msg::builder::Init_RobotPosition_x();
}

}  // namespace radar_station_interface

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__BUILDER_HPP_
