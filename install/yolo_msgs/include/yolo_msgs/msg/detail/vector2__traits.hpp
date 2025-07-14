// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_msgs:msg/Vector2.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__VECTOR2__TRAITS_HPP_
#define YOLO_MSGS__MSG__DETAIL__VECTOR2__TRAITS_HPP_

#include "yolo_msgs/msg/detail/vector2__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::msg::Vector2>()
{
  return "yolo_msgs::msg::Vector2";
}

template<>
inline const char * name<yolo_msgs::msg::Vector2>()
{
  return "yolo_msgs/msg/Vector2";
}

template<>
struct has_fixed_size<yolo_msgs::msg::Vector2>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yolo_msgs::msg::Vector2>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yolo_msgs::msg::Vector2>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_MSGS__MSG__DETAIL__VECTOR2__TRAITS_HPP_
