// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vision_msgs:msg/Classification2D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vision_msgs/msg/detail/classification2_d__rosidl_typesupport_introspection_c.h"
#include "vision_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vision_msgs/msg/detail/classification2_d__functions.h"
#include "vision_msgs/msg/detail/classification2_d__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `results`
#include "vision_msgs/msg/object_hypothesis.h"
// Member `results`
#include "vision_msgs/msg/detail/object_hypothesis__rosidl_typesupport_introspection_c.h"
// Member `source_img`
#include "sensor_msgs/msg/image.h"
// Member `source_img`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Classification2D__rosidl_typesupport_introspection_c__Classification2D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vision_msgs__msg__Classification2D__init(message_memory);
}

void Classification2D__rosidl_typesupport_introspection_c__Classification2D_fini_function(void * message_memory)
{
  vision_msgs__msg__Classification2D__fini(message_memory);
}

size_t Classification2D__rosidl_typesupport_introspection_c__size_function__ObjectHypothesis__results(
  const void * untyped_member)
{
  const vision_msgs__msg__ObjectHypothesis__Sequence * member =
    (const vision_msgs__msg__ObjectHypothesis__Sequence *)(untyped_member);
  return member->size;
}

const void * Classification2D__rosidl_typesupport_introspection_c__get_const_function__ObjectHypothesis__results(
  const void * untyped_member, size_t index)
{
  const vision_msgs__msg__ObjectHypothesis__Sequence * member =
    (const vision_msgs__msg__ObjectHypothesis__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Classification2D__rosidl_typesupport_introspection_c__get_function__ObjectHypothesis__results(
  void * untyped_member, size_t index)
{
  vision_msgs__msg__ObjectHypothesis__Sequence * member =
    (vision_msgs__msg__ObjectHypothesis__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Classification2D__rosidl_typesupport_introspection_c__resize_function__ObjectHypothesis__results(
  void * untyped_member, size_t size)
{
  vision_msgs__msg__ObjectHypothesis__Sequence * member =
    (vision_msgs__msg__ObjectHypothesis__Sequence *)(untyped_member);
  vision_msgs__msg__ObjectHypothesis__Sequence__fini(member);
  return vision_msgs__msg__ObjectHypothesis__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__Classification2D, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "results",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__Classification2D, results),  // bytes offset in struct
    NULL,  // default value
    Classification2D__rosidl_typesupport_introspection_c__size_function__ObjectHypothesis__results,  // size() function pointer
    Classification2D__rosidl_typesupport_introspection_c__get_const_function__ObjectHypothesis__results,  // get_const(index) function pointer
    Classification2D__rosidl_typesupport_introspection_c__get_function__ObjectHypothesis__results,  // get(index) function pointer
    Classification2D__rosidl_typesupport_introspection_c__resize_function__ObjectHypothesis__results  // resize(index) function pointer
  },
  {
    "source_img",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_msgs__msg__Classification2D, source_img),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_members = {
  "vision_msgs__msg",  // message namespace
  "Classification2D",  // message name
  3,  // number of fields
  sizeof(vision_msgs__msg__Classification2D),
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_member_array,  // message members
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_init_function,  // function to initialize message memory (memory has to be allocated)
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_type_support_handle = {
  0,
  &Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vision_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, Classification2D)() {
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, ObjectHypothesis)();
  Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_type_support_handle.typesupport_identifier) {
    Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Classification2D__rosidl_typesupport_introspection_c__Classification2D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
