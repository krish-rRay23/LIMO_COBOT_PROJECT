// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define YOLO_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include "yolo_msgs/msg/detail/detection__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'bbox'
#include "yolo_msgs/msg/detail/bounding_box2_d__traits.hpp"
// Member 'bbox3d'
#include "yolo_msgs/msg/detail/bounding_box3_d__traits.hpp"
// Member 'mask'
#include "yolo_msgs/msg/detail/mask__traits.hpp"
// Member 'keypoints'
#include "yolo_msgs/msg/detail/key_point2_d_array__traits.hpp"
// Member 'keypoints3d'
#include "yolo_msgs/msg/detail/key_point3_d_array__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::msg::Detection>()
{
  return "yolo_msgs::msg::Detection";
}

template<>
inline const char * name<yolo_msgs::msg::Detection>()
{
  return "yolo_msgs/msg/Detection";
}

template<>
struct has_fixed_size<yolo_msgs::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_msgs::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_msgs::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
