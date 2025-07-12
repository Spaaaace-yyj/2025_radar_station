// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_station_interface:msg/RobotPositionArray.idl
// generated code does not contain a copyright notice

#ifndef RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_HPP_
#define RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'positions'
#include "radar_station_interface/msg/detail/robot_position__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_station_interface__msg__RobotPositionArray __attribute__((deprecated))
#else
# define DEPRECATED__radar_station_interface__msg__RobotPositionArray __declspec(deprecated)
#endif

namespace radar_station_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPositionArray_
{
  using Type = RobotPositionArray_<ContainerAllocator>;

  explicit RobotPositionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RobotPositionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _positions_type =
    std::vector<radar_station_interface::msg::RobotPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<radar_station_interface::msg::RobotPosition_<ContainerAllocator>>>;
  _positions_type positions;

  // setters for named parameter idiom
  Type & set__positions(
    const std::vector<radar_station_interface::msg::RobotPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<radar_station_interface::msg::RobotPosition_<ContainerAllocator>>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_station_interface__msg__RobotPositionArray
    std::shared_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_station_interface__msg__RobotPositionArray
    std::shared_ptr<radar_station_interface::msg::RobotPositionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPositionArray_ & other) const
  {
    if (this->positions != other.positions) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPositionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPositionArray_

// alias to use template instance with default allocator
using RobotPositionArray =
  radar_station_interface::msg::RobotPositionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_station_interface

#endif  // RADAR_STATION_INTERFACE__MSG__DETAIL__ROBOT_POSITION_ARRAY__STRUCT_HPP_
