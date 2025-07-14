// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_msgs:srv/SetClasses.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_msgs/srv/detail/set_classes__rosidl_typesupport_introspection_c.h"
#include "yolo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_msgs/srv/detail/set_classes__functions.h"
#include "yolo_msgs/srv/detail/set_classes__struct.h"


// Include directives for member types
// Member `classes`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_msgs__srv__SetClasses_Request__init(message_memory);
}

void SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_fini_function(void * message_memory)
{
  yolo_msgs__srv__SetClasses_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_member_array[1] = {
  {
    "classes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msgs__srv__SetClasses_Request, classes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_members = {
  "yolo_msgs__srv",  // message namespace
  "SetClasses_Request",  // message name
  1,  // number of fields
  sizeof(yolo_msgs__srv__SetClasses_Request),
  SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_member_array,  // message members
  SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_type_support_handle = {
  0,
  &SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Request)() {
  if (!SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_type_support_handle.typesupport_identifier) {
    SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetClasses_Request__rosidl_typesupport_introspection_c__SetClasses_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "yolo_msgs/srv/detail/set_classes__rosidl_typesupport_introspection_c.h"
// already included above
// #include "yolo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "yolo_msgs/srv/detail/set_classes__functions.h"
// already included above
// #include "yolo_msgs/srv/detail/set_classes__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_msgs__srv__SetClasses_Response__init(message_memory);
}

void SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_fini_function(void * message_memory)
{
  yolo_msgs__srv__SetClasses_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msgs__srv__SetClasses_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_members = {
  "yolo_msgs__srv",  // message namespace
  "SetClasses_Response",  // message name
  1,  // number of fields
  sizeof(yolo_msgs__srv__SetClasses_Response),
  SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_member_array,  // message members
  SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_type_support_handle = {
  0,
  &SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Response)() {
  if (!SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_type_support_handle.typesupport_identifier) {
    SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetClasses_Response__rosidl_typesupport_introspection_c__SetClasses_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "yolo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "yolo_msgs/srv/detail/set_classes__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_members = {
  "yolo_msgs__srv",  // service namespace
  "SetClasses",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_Request_message_type_support_handle,
  NULL  // response message
  // yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_Response_message_type_support_handle
};

static rosidl_service_type_support_t yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_type_support_handle = {
  0,
  &yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses)() {
  if (!yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_type_support_handle.typesupport_identifier) {
    yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, srv, SetClasses_Response)()->data;
  }

  return &yolo_msgs__srv__detail__set_classes__rosidl_typesupport_introspection_c__SetClasses_service_type_support_handle;
}
