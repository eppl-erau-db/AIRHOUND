// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:srv/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/srv/vehicle_command.h"


#ifndef PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__STRUCT_H_
#define PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "px4_msgs/msg/detail/vehicle_command__struct.h"

/// Struct defined in srv/VehicleCommand in the package px4_msgs.
typedef struct px4_msgs__srv__VehicleCommand_Request
{
  px4_msgs__msg__VehicleCommand request;
} px4_msgs__srv__VehicleCommand_Request;

// Struct for a sequence of px4_msgs__srv__VehicleCommand_Request.
typedef struct px4_msgs__srv__VehicleCommand_Request__Sequence
{
  px4_msgs__srv__VehicleCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__srv__VehicleCommand_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'reply'
#include "px4_msgs/msg/detail/vehicle_command_ack__struct.h"

/// Struct defined in srv/VehicleCommand in the package px4_msgs.
typedef struct px4_msgs__srv__VehicleCommand_Response
{
  px4_msgs__msg__VehicleCommandAck reply;
} px4_msgs__srv__VehicleCommand_Response;

// Struct for a sequence of px4_msgs__srv__VehicleCommand_Response.
typedef struct px4_msgs__srv__VehicleCommand_Response__Sequence
{
  px4_msgs__srv__VehicleCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__srv__VehicleCommand_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  px4_msgs__srv__VehicleCommand_Event__request__MAX_SIZE = 1
};
// response
enum
{
  px4_msgs__srv__VehicleCommand_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/VehicleCommand in the package px4_msgs.
typedef struct px4_msgs__srv__VehicleCommand_Event
{
  service_msgs__msg__ServiceEventInfo info;
  px4_msgs__srv__VehicleCommand_Request__Sequence request;
  px4_msgs__srv__VehicleCommand_Response__Sequence response;
} px4_msgs__srv__VehicleCommand_Event;

// Struct for a sequence of px4_msgs__srv__VehicleCommand_Event.
typedef struct px4_msgs__srv__VehicleCommand_Event__Sequence
{
  px4_msgs__srv__VehicleCommand_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__srv__VehicleCommand_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__STRUCT_H_
