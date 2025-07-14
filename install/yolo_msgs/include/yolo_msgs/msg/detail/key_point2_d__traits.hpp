// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_msgs:msg/KeyPoint2D.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__KEY_POINT2_D__TRAITS_HPP_
#define YOLO_MSGS__MSG__DETAIL__KEY_POINT2_D__TRAITS_HPP_

#include "yolo_msgs/msg/detail/key_point2_d__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'point'
#include "yolo_msgs/msg/detail/point2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::msg::KeyPoint2D>()
{
  return "yolo_msgs::msg::KeyPoint2D";
}

template<>
inline const char * name<yolo_msgs::msg::KeyPoint2D>()
{
  return "yolo_msgs/msg/KeyPoint2D";
}

template<>
struct has_fixed_size<yolo_msgs::msg::KeyPoint2D>
  : std::integral_constant<bool, has_fixed_size<yolo_msgs::msg::Point2D>::value> {};

template<>
struct has_bounded_size<yolo_msgs::msg::KeyPoint2D>
  : std::integral_constant<bool, has_bounded_size<yolo_msgs::msg::Point2D>::value> {};

template<>
struct is_message<yolo_msgs::msg::KeyPoint2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_MSGS__MSG__DETAIL__KEY_POINT2_D__TRAITS_HPP_
