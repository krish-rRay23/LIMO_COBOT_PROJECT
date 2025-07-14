// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vision_msgs:msg/BoundingBox2DArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vision_msgs/msg/detail/bounding_box2_d_array__rosidl_typesupport_introspection_c.h"
#include "vision_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vision_msgs/msg/detail/bounding_box2_d_array__functions.h"
#include "vision_msgs/msg/detail/bounding_box2_d_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `boxes`
#include "vision_msgs/msg/bounding_box2_d.h"
// Member `boxes`
#include "vision_msgs/msg/detail/bounding_box2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vision_msgs__msg__BoundingBox2DArray__init(message_memory);
}

void BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_fini_function(void * message_memory)
{
  vision_msgs__msg__BoundingBox2DArray__fini(message_memory);
}

size_t BoundingBox2DArray__rosidl_typesupport_introspection_c__size_function__BoundingBox2D__boxes(
  const void * untyped_member)
{
  const vision_msgs__msg__BoundingBox2D__Sequence * member =
    (const vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return member->size;
}

const void * BoundingBox2DArray__rosidl_typesupport_introspection_c__get_const_function__BoundingBox2D__boxes(
  const void * untyped_member, size_t index)
{
  const vision_msgs__msg__BoundingBox2D__Sequence * member =
    (const vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * BoundingBox2DArray__rosidl_typesupport_introspection_c__get_function__BoundingBox2D__boxes(
  void * untyped_member, size_t index)
{
  vision_msgs__msg__BoundingBox2D__Sequence * member =
    (vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return &member->data[index];
}

bool BoundingBox2DArray__rosidl_typesupport_introspection_c__resize_function__BoundingBox2D__boxes(
  void * untyped_member, size_t size)
{
  vision_msgs__msg__BoundingBox2D__Sequence * member =
    (vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  vision_msgs__msg__BoundingBox2D__Sequence__fini(member);
  return vision_msgs__msg__BoundingBox2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__BoundingBox2DArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__BoundingBox2DArray, boxes),  // bytes offset in struct
    NULL,  // default value
    BoundingBox2DArray__rosidl_typesupport_introspection_c__size_function__BoundingBox2D__boxes,  // size() function pointer
    BoundingBox2DArray__rosidl_typesupport_introspection_c__get_const_function__BoundingBox2D__boxes,  // get_const(index) function pointer
    BoundingBox2DArray__rosidl_typesupport_introspection_c__get_function__BoundingBox2D__boxes,  // get(index) function pointer
    BoundingBox2DArray__rosidl_typesupport_introspection_c__resize_function__BoundingBox2D__boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_members = {
  "vision_msgs__msg",  // message namespace
  "BoundingBox2DArray",  // message name
  2,  // number of fields
  sizeof(vision_msgs__msg__BoundingBox2DArray),
  BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_member_array,  // message members
  BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_init_function,  // function to initialize message memory (memory has to be allocated)
  BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_type_support_handle = {
  0,
  &BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vision_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, BoundingBox2DArray)() {
  BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, BoundingBox2D)();
  if (!BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_type_support_handle.typesupport_identifier) {
    BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BoundingBox2DArray__rosidl_typesupport_introspection_c__BoundingBox2DArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
