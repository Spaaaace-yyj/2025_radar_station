// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_station_interface:msg/RobotPosition.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__radar_station_interface__msg__RobotPosition __attribute__((deprecated))
#else
# define DEPRECATED__radar_station_interface__msg__RobotPosition __declspec(deprecated)
#endif

namespace radar_station_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPosition_
{
  using Type = RobotPosition_<ContainerAllocator>;

  explicit RobotPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->width = 0.0;
      this->height = 0.0;
      this->depth = 0.0;
      this->id = 0l;
    }
  }

  explicit RobotPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->width = 0.0;
      this->height = 0.0;
      this->depth = 0.0;
      this->id = 0l;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _width_type =
    double;
  _width_type width;
  using _height_type =
    double;
  _height_type height;
  using _depth_type =
    double;
  _depth_type depth;
  using _id_type =
    int32_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__width(
    const double & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const double & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__depth(
    const double & _arg)
  {
    this->depth = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_station_interface::msg::RobotPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_station_interface::msg::RobotPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_station_interface::msg::RobotPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_station_interface::msg::RobotPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_station_interface__msg__RobotPosition
    std::shared_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_station_interface__msg__RobotPosition
    std::shared_ptr<radar_station_interface::msg::RobotPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPosition_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPosition_

// alias to use template instance with default allocator
using RobotPosition =
  radar_station_interface::msg::RobotPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_station_interface

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION__STRUCT_HPP_
