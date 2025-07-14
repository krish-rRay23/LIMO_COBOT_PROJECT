// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/Classification2D.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__CLASSIFICATION2_D__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__CLASSIFICATION2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'results'
#include "vision_msgs/msg/detail/object_hypothesis__struct.h"
// Member 'source_img'
#include "sensor_msgs/msg/detail/image__struct.h"

// Struct defined in msg/Classification2D in the package vision_msgs.
typedef struct vision_msgs__msg__Classification2D
{
  std_msgs__msg__Header header;
  vision_msgs__msg__ObjectHypothesis__Sequence results;
  sensor_msgs__msg__Image source_img;
} vision_msgs__msg__Classification2D;

// Struct for a sequence of vision_msgs__msg__Classification2D.
typedef struct vision_msgs__msg__Classification2D__Sequence
{
  vision_msgs__msg__Classification2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__Classification2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__CLASSIFICATION2_D__STRUCT_H_
