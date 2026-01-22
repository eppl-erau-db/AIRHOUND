// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/GimbalDeviceInformation.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/gimbal_device_information__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/gimbal_device_information__struct.h"
#include "px4_msgs/msg/detail/gimbal_device_information__functions.h"
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


using _GimbalDeviceInformation__ros_msg_type = px4_msgs__msg__GimbalDeviceInformation;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__GimbalDeviceInformation(
  const px4_msgs__msg__GimbalDeviceInformation * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: vendor_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->vendor_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: model_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->model_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: custom_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->custom_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: firmware_version
  {
    cdr << ros_message->firmware_version;
  }

  // Field name: hardware_version
  {
    cdr << ros_message->hardware_version;
  }

  // Field name: uid
  {
    cdr << ros_message->uid;
  }

  // Field name: cap_flags
  {
    cdr << ros_message->cap_flags;
  }

  // Field name: custom_cap_flags
  {
    cdr << ros_message->custom_cap_flags;
  }

  // Field name: roll_min
  {
    cdr << ros_message->roll_min;
  }

  // Field name: roll_max
  {
    cdr << ros_message->roll_max;
  }

  // Field name: pitch_min
  {
    cdr << ros_message->pitch_min;
  }

  // Field name: pitch_max
  {
    cdr << ros_message->pitch_max;
  }

  // Field name: yaw_min
  {
    cdr << ros_message->yaw_min;
  }

  // Field name: yaw_max
  {
    cdr << ros_message->yaw_max;
  }

  // Field name: gimbal_device_id
  {
    cdr << ros_message->gimbal_device_id;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__GimbalDeviceInformation(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__GimbalDeviceInformation * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: vendor_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->vendor_name;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: model_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->model_name;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: custom_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->custom_name;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: firmware_version
  {
    cdr >> ros_message->firmware_version;
  }

  // Field name: hardware_version
  {
    cdr >> ros_message->hardware_version;
  }

  // Field name: uid
  {
    cdr >> ros_message->uid;
  }

  // Field name: cap_flags
  {
    cdr >> ros_message->cap_flags;
  }

  // Field name: custom_cap_flags
  {
    cdr >> ros_message->custom_cap_flags;
  }

  // Field name: roll_min
  {
    cdr >> ros_message->roll_min;
  }

  // Field name: roll_max
  {
    cdr >> ros_message->roll_max;
  }

  // Field name: pitch_min
  {
    cdr >> ros_message->pitch_min;
  }

  // Field name: pitch_max
  {
    cdr >> ros_message->pitch_max;
  }

  // Field name: yaw_min
  {
    cdr >> ros_message->yaw_min;
  }

  // Field name: yaw_max
  {
    cdr >> ros_message->yaw_max;
  }

  // Field name: gimbal_device_id
  {
    cdr >> ros_message->gimbal_device_id;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__GimbalDeviceInformation(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GimbalDeviceInformation__ros_msg_type * ros_message = static_cast<const _GimbalDeviceInformation__ros_msg_type *>(untyped_ros_message);
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

  // Field name: vendor_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->vendor_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: model_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->model_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: custom_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->custom_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: firmware_version
  {
    size_t item_size = sizeof(ros_message->firmware_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hardware_version
  {
    size_t item_size = sizeof(ros_message->hardware_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: uid
  {
    size_t item_size = sizeof(ros_message->uid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cap_flags
  {
    size_t item_size = sizeof(ros_message->cap_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: custom_cap_flags
  {
    size_t item_size = sizeof(ros_message->custom_cap_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: roll_min
  {
    size_t item_size = sizeof(ros_message->roll_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: roll_max
  {
    size_t item_size = sizeof(ros_message->roll_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pitch_min
  {
    size_t item_size = sizeof(ros_message->pitch_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pitch_max
  {
    size_t item_size = sizeof(ros_message->pitch_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_min
  {
    size_t item_size = sizeof(ros_message->yaw_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_max
  {
    size_t item_size = sizeof(ros_message->yaw_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gimbal_device_id
  {
    size_t item_size = sizeof(ros_message->gimbal_device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__GimbalDeviceInformation(
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

  // Field name: vendor_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: model_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: custom_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: firmware_version
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hardware_version
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: uid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: cap_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: custom_cap_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: roll_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: roll_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pitch_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pitch_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gimbal_device_id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__GimbalDeviceInformation;
    is_plain =
      (
      offsetof(DataType, gimbal_device_id) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__GimbalDeviceInformation(
  const px4_msgs__msg__GimbalDeviceInformation * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: vendor_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->vendor_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: model_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->model_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: custom_name
  {
    size_t size = 32;
    auto array_ptr = ros_message->custom_name;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: firmware_version
  {
    cdr << ros_message->firmware_version;
  }

  // Field name: hardware_version
  {
    cdr << ros_message->hardware_version;
  }

  // Field name: uid
  {
    cdr << ros_message->uid;
  }

  // Field name: cap_flags
  {
    cdr << ros_message->cap_flags;
  }

  // Field name: custom_cap_flags
  {
    cdr << ros_message->custom_cap_flags;
  }

  // Field name: roll_min
  {
    cdr << ros_message->roll_min;
  }

  // Field name: roll_max
  {
    cdr << ros_message->roll_max;
  }

  // Field name: pitch_min
  {
    cdr << ros_message->pitch_min;
  }

  // Field name: pitch_max
  {
    cdr << ros_message->pitch_max;
  }

  // Field name: yaw_min
  {
    cdr << ros_message->yaw_min;
  }

  // Field name: yaw_max
  {
    cdr << ros_message->yaw_max;
  }

  // Field name: gimbal_device_id
  {
    cdr << ros_message->gimbal_device_id;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__GimbalDeviceInformation(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GimbalDeviceInformation__ros_msg_type * ros_message = static_cast<const _GimbalDeviceInformation__ros_msg_type *>(untyped_ros_message);
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

  // Field name: vendor_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->vendor_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: model_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->model_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: custom_name
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->custom_name;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: firmware_version
  {
    size_t item_size = sizeof(ros_message->firmware_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hardware_version
  {
    size_t item_size = sizeof(ros_message->hardware_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: uid
  {
    size_t item_size = sizeof(ros_message->uid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cap_flags
  {
    size_t item_size = sizeof(ros_message->cap_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: custom_cap_flags
  {
    size_t item_size = sizeof(ros_message->custom_cap_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: roll_min
  {
    size_t item_size = sizeof(ros_message->roll_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: roll_max
  {
    size_t item_size = sizeof(ros_message->roll_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pitch_min
  {
    size_t item_size = sizeof(ros_message->pitch_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pitch_max
  {
    size_t item_size = sizeof(ros_message->pitch_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_min
  {
    size_t item_size = sizeof(ros_message->yaw_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_max
  {
    size_t item_size = sizeof(ros_message->yaw_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gimbal_device_id
  {
    size_t item_size = sizeof(ros_message->gimbal_device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__GimbalDeviceInformation(
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

  // Field name: vendor_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: model_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: custom_name
  {
    size_t array_size = 32;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: firmware_version
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hardware_version
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: uid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: cap_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: custom_cap_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: roll_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: roll_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pitch_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pitch_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_min
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gimbal_device_id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__GimbalDeviceInformation;
    is_plain =
      (
      offsetof(DataType, gimbal_device_id) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _GimbalDeviceInformation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__GimbalDeviceInformation * ros_message = static_cast<const px4_msgs__msg__GimbalDeviceInformation *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__GimbalDeviceInformation(ros_message, cdr);
}

static bool _GimbalDeviceInformation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__GimbalDeviceInformation * ros_message = static_cast<px4_msgs__msg__GimbalDeviceInformation *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__GimbalDeviceInformation(cdr, ros_message);
}

static uint32_t _GimbalDeviceInformation__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__GimbalDeviceInformation(
      untyped_ros_message, 0));
}

static size_t _GimbalDeviceInformation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__GimbalDeviceInformation(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GimbalDeviceInformation = {
  "px4_msgs::msg",
  "GimbalDeviceInformation",
  _GimbalDeviceInformation__cdr_serialize,
  _GimbalDeviceInformation__cdr_deserialize,
  _GimbalDeviceInformation__get_serialized_size,
  _GimbalDeviceInformation__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _GimbalDeviceInformation__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GimbalDeviceInformation,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__GimbalDeviceInformation__get_type_hash,
  &px4_msgs__msg__GimbalDeviceInformation__get_type_description,
  &px4_msgs__msg__GimbalDeviceInformation__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, GimbalDeviceInformation)() {
  return &_GimbalDeviceInformation__type_support;
}

#if defined(__cplusplus)
}
#endif
