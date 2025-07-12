// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__TRAITS_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_station_interface/msg/detail/robot_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace radar_station_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPosition & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: depth
  {
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotPosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotPosition & msg, bool use_flow_style = false)
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
  const radar_station_interface::msg::RobotPosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_station_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_station_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_station_interface::msg::RobotPosition & msg)
{
  return radar_station_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_station_interface::msg::RobotPosition>()
{
  return "radar_station_interface::msg::RobotPosition";
}

template<>
inline const char * name<radar_station_interface::msg::RobotPosition>()
{
  return "radar_station_interface/msg/RobotPosition";
}

template<>
struct has_fixed_size<radar_station_interface::msg::RobotPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<radar_station_interface::msg::RobotPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<radar_station_interface::msg::RobotPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__TRAITS_HPP_
