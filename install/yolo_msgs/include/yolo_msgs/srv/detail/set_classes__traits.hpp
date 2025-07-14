// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_msgs:srv/SetClasses.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__SRV__DETAIL__SET_CLASSES__TRAITS_HPP_
#define YOLO_MSGS__SRV__DETAIL__SET_CLASSES__TRAITS_HPP_

#include "yolo_msgs/srv/detail/set_classes__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::srv::SetClasses_Request>()
{
  return "yolo_msgs::srv::SetClasses_Request";
}

template<>
inline const char * name<yolo_msgs::srv::SetClasses_Request>()
{
  return "yolo_msgs/srv/SetClasses_Request";
}

template<>
struct has_fixed_size<yolo_msgs::srv::SetClasses_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_msgs::srv::SetClasses_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_msgs::srv::SetClasses_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::srv::SetClasses_Response>()
{
  return "yolo_msgs::srv::SetClasses_Response";
}

template<>
inline const char * name<yolo_msgs::srv::SetClasses_Response>()
{
  return "yolo_msgs/srv/SetClasses_Response";
}

template<>
struct has_fixed_size<yolo_msgs::srv::SetClasses_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yolo_msgs::srv::SetClasses_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yolo_msgs::srv::SetClasses_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::srv::SetClasses>()
{
  return "yolo_msgs::srv::SetClasses";
}

template<>
inline const char * name<yolo_msgs::srv::SetClasses>()
{
  return "yolo_msgs/srv/SetClasses";
}

template<>
struct has_fixed_size<yolo_msgs::srv::SetClasses>
  : std::integral_constant<
    bool,
    has_fixed_size<yolo_msgs::srv::SetClasses_Request>::value &&
    has_fixed_size<yolo_msgs::srv::SetClasses_Response>::value
  >
{
};

template<>
struct has_bounded_size<yolo_msgs::srv::SetClasses>
  : std::integral_constant<
    bool,
    has_bounded_size<yolo_msgs::srv::SetClasses_Request>::value &&
    has_bounded_size<yolo_msgs::srv::SetClasses_Response>::value
  >
{
};

template<>
struct is_service<yolo_msgs::srv::SetClasses>
  : std::true_type
{
};

template<>
struct is_service_request<yolo_msgs::srv::SetClasses_Request>
  : std::true_type
{
};

template<>
struct is_service_response<yolo_msgs::srv::SetClasses_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // YOLO_MSGS__SRV__DETAIL__SET_CLASSES__TRAITS_HPP_
