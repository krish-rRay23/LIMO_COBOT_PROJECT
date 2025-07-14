// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/VisionInfo.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__VISION_INFO__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__VISION_INFO__STRUCT_H_

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
// Member 'method'
// Member 'database_location'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VisionInfo in the package vision_msgs.
typedef struct vision_msgs__msg__VisionInfo
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String method;
  rosidl_runtime_c__String database_location;
  int32_t database_version;
} vision_msgs__msg__VisionInfo;

// Struct for a sequence of vision_msgs__msg__VisionInfo.
typedef struct vision_msgs__msg__VisionInfo__Sequence
{
  vision_msgs__msg__VisionInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__VisionInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__VISION_INFO__STRUCT_H_
