// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_patrol:srv/GetDirection.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_PATROL__SRV__DETAIL__GET_DIRECTION__TRAITS_HPP_
#define ROBOT_PATROL__SRV__DETAIL__GET_DIRECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_patrol/srv/detail/get_direction__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'laser_data'
#include "sensor_msgs/msg/detail/laser_scan__traits.hpp"

namespace robot_patrol
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetDirection_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: laser_data
  {
    out << "laser_data: ";
    to_flow_style_yaml(msg.laser_data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetDirection_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: laser_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "laser_data:\n";
    to_block_style_yaml(msg.laser_data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetDirection_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_patrol

namespace rosidl_generator_traits
{

[[deprecated("use robot_patrol::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_patrol::srv::GetDirection_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_patrol::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_patrol::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_patrol::srv::GetDirection_Request & msg)
{
  return robot_patrol::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_patrol::srv::GetDirection_Request>()
{
  return "robot_patrol::srv::GetDirection_Request";
}

template<>
inline const char * name<robot_patrol::srv::GetDirection_Request>()
{
  return "robot_patrol/srv/GetDirection_Request";
}

template<>
struct has_fixed_size<robot_patrol::srv::GetDirection_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::LaserScan>::value> {};

template<>
struct has_bounded_size<robot_patrol::srv::GetDirection_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::LaserScan>::value> {};

template<>
struct is_message<robot_patrol::srv::GetDirection_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_patrol
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetDirection_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetDirection_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetDirection_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_patrol

namespace rosidl_generator_traits
{

[[deprecated("use robot_patrol::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_patrol::srv::GetDirection_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_patrol::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_patrol::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_patrol::srv::GetDirection_Response & msg)
{
  return robot_patrol::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_patrol::srv::GetDirection_Response>()
{
  return "robot_patrol::srv::GetDirection_Response";
}

template<>
inline const char * name<robot_patrol::srv::GetDirection_Response>()
{
  return "robot_patrol/srv/GetDirection_Response";
}

template<>
struct has_fixed_size<robot_patrol::srv::GetDirection_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_patrol::srv::GetDirection_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_patrol::srv::GetDirection_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_patrol::srv::GetDirection>()
{
  return "robot_patrol::srv::GetDirection";
}

template<>
inline const char * name<robot_patrol::srv::GetDirection>()
{
  return "robot_patrol/srv/GetDirection";
}

template<>
struct has_fixed_size<robot_patrol::srv::GetDirection>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_patrol::srv::GetDirection_Request>::value &&
    has_fixed_size<robot_patrol::srv::GetDirection_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_patrol::srv::GetDirection>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_patrol::srv::GetDirection_Request>::value &&
    has_bounded_size<robot_patrol::srv::GetDirection_Response>::value
  >
{
};

template<>
struct is_service<robot_patrol::srv::GetDirection>
  : std::true_type
{
};

template<>
struct is_service_request<robot_patrol::srv::GetDirection_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_patrol::srv::GetDirection_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_PATROL__SRV__DETAIL__GET_DIRECTION__TRAITS_HPP_
