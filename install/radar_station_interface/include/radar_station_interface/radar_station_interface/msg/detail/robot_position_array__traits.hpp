// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__TRAITS_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_station_interface/msg/detail/robot_position_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'positions'
#include "radar_station_interface/msg/detail/robot_position__traits.hpp"

namespace radar_station_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPositionArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotPositionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotPositionArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace radar_station_interface

namespace rosidl_generator_traits
{

[[deprecated("use radar_station_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_station_interface::msg::RobotPositionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_station_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_station_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_station_interface::msg::RobotPositionArray & msg)
{
  return radar_station_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_station_interface::msg::RobotPositionArray>()
{
  return "radar_station_interface::msg::RobotPositionArray";
}

template<>
inline const char * name<radar_station_interface::msg::RobotPositionArray>()
{
  return "radar_station_interface/msg/RobotPositionArray";
}

template<>
struct has_fixed_size<radar_station_interface::msg::RobotPositionArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<radar_station_interface::msg::RobotPositionArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<radar_station_interface::msg::RobotPositionArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__TRAITS_HPP_
