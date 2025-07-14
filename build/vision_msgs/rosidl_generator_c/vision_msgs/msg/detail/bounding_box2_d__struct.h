// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/BoundingBox2D.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__BOUNDING_BOX2_D__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__BOUNDING_BOX2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'center'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

// Struct defined in msg/BoundingBox2D in the package vision_msgs.
typedef struct vision_msgs__msg__BoundingBox2D
{
  geometry_msgs__msg__Pose2D center;
  double size_x;
  double size_y;
} vision_msgs__msg__BoundingBox2D;

// Struct for a sequence of vision_msgs__msg__BoundingBox2D.
typedef struct vision_msgs__msg__BoundingBox2D__Sequence
{
  vision_msgs__msg__BoundingBox2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__BoundingBox2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__BOUNDING_BOX2_D__STRUCT_H_
