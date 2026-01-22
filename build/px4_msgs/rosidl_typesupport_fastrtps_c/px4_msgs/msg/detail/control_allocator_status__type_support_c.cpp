// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/ControlAllocatorStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/control_allocator_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/control_allocator_status__struct.h"
#include "px4_msgs/msg/detail/control_allocator_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ControlAllocatorStatus__ros_msg_type = px4_msgs__msg__ControlAllocatorStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__ControlAllocatorStatus(
  const px4_msgs__msg__ControlAllocatorStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: torque_setpoint_achieved
  {
    cdr << (ros_message->torque_setpoint_achieved ? true : false);
  }

  // Field name: unallocated_torque
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_torque;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: thrust_setpoint_achieved
  {
    cdr << (ros_message->thrust_setpoint_achieved ? true : false);
  }

  // Field name: unallocated_thrust
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_thrust;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: actuator_saturation
  {
    size_t size = 16;
    auto array_ptr = ros_message->actuator_saturation;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: handled_motor_failure_mask
  {
    cdr << ros_message->handled_motor_failure_mask;
  }

  // Field name: motor_stop_mask
  {
    cdr << ros_message->motor_stop_mask;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__ControlAllocatorStatus(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__ControlAllocatorStatus * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: torque_setpoint_achieved
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->torque_setpoint_achieved = tmp ? true : false;
  }

  // Field name: unallocated_torque
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_torque;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: thrust_setpoint_achieved
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->thrust_setpoint_achieved = tmp ? true : false;
  }

  // Field name: unallocated_thrust
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_thrust;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: actuator_saturation
  {
    size_t size = 16;
    auto array_ptr = ros_message->actuator_saturation;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: handled_motor_failure_mask
  {
    cdr >> ros_message->handled_motor_failure_mask;
  }

  // Field name: motor_stop_mask
  {
    cdr >> ros_message->motor_stop_mask;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__ControlAllocatorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ControlAllocatorStatus__ros_msg_type * ros_message = static_cast<const _ControlAllocatorStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: torque_setpoint_achieved
  {
    size_t item_size = sizeof(ros_message->torque_setpoint_achieved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: unallocated_torque
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->unallocated_torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: thrust_setpoint_achieved
  {
    size_t item_size = sizeof(ros_message->thrust_setpoint_achieved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: unallocated_thrust
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->unallocated_thrust;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: actuator_saturation
  {
    size_t array_size = 16;
    auto array_ptr = ros_message->actuator_saturation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: handled_motor_failure_mask
  {
    size_t item_size = sizeof(ros_message->handled_motor_failure_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_stop_mask
  {
    size_t item_size = sizeof(ros_message->motor_stop_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__ControlAllocatorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: timestamp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: torque_setpoint_achieved
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: unallocated_torque
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: thrust_setpoint_achieved
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: unallocated_thrust
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: actuator_saturation
  {
    size_t array_size = 16;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: handled_motor_failure_mask
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: motor_stop_mask
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__ControlAllocatorStatus;
    is_plain =
      (
      offsetof(DataType, motor_stop_mask) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__ControlAllocatorStatus(
  const px4_msgs__msg__ControlAllocatorStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: torque_setpoint_achieved
  {
    cdr << (ros_message->torque_setpoint_achieved ? true : false);
  }

  // Field name: unallocated_torque
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_torque;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: thrust_setpoint_achieved
  {
    cdr << (ros_message->thrust_setpoint_achieved ? true : false);
  }

  // Field name: unallocated_thrust
  {
    size_t size = 3;
    auto array_ptr = ros_message->unallocated_thrust;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: actuator_saturation
  {
    size_t size = 16;
    auto array_ptr = ros_message->actuator_saturation;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: handled_motor_failure_mask
  {
    cdr << ros_message->handled_motor_failure_mask;
  }

  // Field name: motor_stop_mask
  {
    cdr << ros_message->motor_stop_mask;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__ControlAllocatorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ControlAllocatorStatus__ros_msg_type * ros_message = static_cast<const _ControlAllocatorStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: torque_setpoint_achieved
  {
    size_t item_size = sizeof(ros_message->torque_setpoint_achieved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: unallocated_torque
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->unallocated_torque;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: thrust_setpoint_achieved
  {
    size_t item_size = sizeof(ros_message->thrust_setpoint_achieved);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: unallocated_thrust
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->unallocated_thrust;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: actuator_saturation
  {
    size_t array_size = 16;
    auto array_ptr = ros_message->actuator_saturation;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: handled_motor_failure_mask
  {
    size_t item_size = sizeof(ros_message->handled_motor_failure_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_stop_mask
  {
    size_t item_size = sizeof(ros_message->motor_stop_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__ControlAllocatorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: timestamp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: torque_setpoint_achieved
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: unallocated_torque
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: thrust_setpoint_achieved
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: unallocated_thrust
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: actuator_saturation
  {
    size_t array_size = 16;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: handled_motor_failure_mask
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: motor_stop_mask
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__ControlAllocatorStatus;
    is_plain =
      (
      offsetof(DataType, motor_stop_mask) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ControlAllocatorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__ControlAllocatorStatus * ros_message = static_cast<const px4_msgs__msg__ControlAllocatorStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__ControlAllocatorStatus(ros_message, cdr);
}

static bool _ControlAllocatorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__ControlAllocatorStatus * ros_message = static_cast<px4_msgs__msg__ControlAllocatorStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__ControlAllocatorStatus(cdr, ros_message);
}

static uint32_t _ControlAllocatorStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__ControlAllocatorStatus(
      untyped_ros_message, 0));
}

static size_t _ControlAllocatorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__ControlAllocatorStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ControlAllocatorStatus = {
  "px4_msgs::msg",
  "ControlAllocatorStatus",
  _ControlAllocatorStatus__cdr_serialize,
  _ControlAllocatorStatus__cdr_deserialize,
  _ControlAllocatorStatus__get_serialized_size,
  _ControlAllocatorStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ControlAllocatorStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ControlAllocatorStatus,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__ControlAllocatorStatus__get_type_hash,
  &px4_msgs__msg__ControlAllocatorStatus__get_type_description,
  &px4_msgs__msg__ControlAllocatorStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, ControlAllocatorStatus)() {
  return &_ControlAllocatorStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
