// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/SensorsStatusImu.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensors_status_imu__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/sensors_status_imu__struct.h"
#include "px4_msgs/msg/detail/sensors_status_imu__functions.h"
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


using _SensorsStatusImu__ros_msg_type = px4_msgs__msg__SensorsStatusImu;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__SensorsStatusImu(
  const px4_msgs__msg__SensorsStatusImu * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: accel_device_id_primary
  {
    cdr << ros_message->accel_device_id_primary;
  }

  // Field name: accel_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_device_ids;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_inconsistency_m_s_s;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_healthy;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_priority;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_device_id_primary
  {
    cdr << ros_message->gyro_device_id_primary;
  }

  // Field name: gyro_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_device_ids;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_inconsistency_rad_s;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_healthy;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_priority;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__SensorsStatusImu(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__SensorsStatusImu * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: accel_device_id_primary
  {
    cdr >> ros_message->accel_device_id_primary;
  }

  // Field name: accel_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_device_ids;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_inconsistency_m_s_s;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: accel_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_healthy;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: accel_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_priority;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gyro_device_id_primary
  {
    cdr >> ros_message->gyro_device_id_primary;
  }

  // Field name: gyro_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_device_ids;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_inconsistency_rad_s;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gyro_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_healthy;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: gyro_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_priority;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__SensorsStatusImu(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SensorsStatusImu__ros_msg_type * ros_message = static_cast<const _SensorsStatusImu__ros_msg_type *>(untyped_ros_message);
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

  // Field name: accel_device_id_primary
  {
    size_t item_size = sizeof(ros_message->accel_device_id_primary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_device_ids
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_device_ids;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_inconsistency_m_s_s;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_healthy
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_healthy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_priority
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_priority;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_device_id_primary
  {
    size_t item_size = sizeof(ros_message->gyro_device_id_primary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_device_ids
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_device_ids;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_inconsistency_rad_s;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_healthy
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_healthy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_priority
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_priority;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__SensorsStatusImu(
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

  // Field name: accel_device_id_primary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_device_ids
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_healthy
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: accel_priority
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: gyro_device_id_primary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_device_ids
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_healthy
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: gyro_priority
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__SensorsStatusImu;
    is_plain =
      (
      offsetof(DataType, gyro_priority) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__SensorsStatusImu(
  const px4_msgs__msg__SensorsStatusImu * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: accel_device_id_primary
  {
    cdr << ros_message->accel_device_id_primary;
  }

  // Field name: accel_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_device_ids;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_inconsistency_m_s_s;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_healthy;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: accel_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->accel_priority;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_device_id_primary
  {
    cdr << ros_message->gyro_device_id_primary;
  }

  // Field name: gyro_device_ids
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_device_ids;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_inconsistency_rad_s;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_healthy
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_healthy;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gyro_priority
  {
    size_t size = 4;
    auto array_ptr = ros_message->gyro_priority;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__SensorsStatusImu(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SensorsStatusImu__ros_msg_type * ros_message = static_cast<const _SensorsStatusImu__ros_msg_type *>(untyped_ros_message);
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

  // Field name: accel_device_id_primary
  {
    size_t item_size = sizeof(ros_message->accel_device_id_primary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_device_ids
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_device_ids;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_inconsistency_m_s_s;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_healthy
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_healthy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: accel_priority
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->accel_priority;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_device_id_primary
  {
    size_t item_size = sizeof(ros_message->gyro_device_id_primary);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_device_ids
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_device_ids;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_inconsistency_rad_s;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_healthy
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_healthy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gyro_priority
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->gyro_priority;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__SensorsStatusImu(
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

  // Field name: accel_device_id_primary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_device_ids
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_inconsistency_m_s_s
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: accel_healthy
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: accel_priority
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: gyro_device_id_primary
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_device_ids
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_inconsistency_rad_s
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gyro_healthy
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: gyro_priority
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__SensorsStatusImu;
    is_plain =
      (
      offsetof(DataType, gyro_priority) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SensorsStatusImu__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__SensorsStatusImu * ros_message = static_cast<const px4_msgs__msg__SensorsStatusImu *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__SensorsStatusImu(ros_message, cdr);
}

static bool _SensorsStatusImu__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__SensorsStatusImu * ros_message = static_cast<px4_msgs__msg__SensorsStatusImu *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__SensorsStatusImu(cdr, ros_message);
}

static uint32_t _SensorsStatusImu__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__SensorsStatusImu(
      untyped_ros_message, 0));
}

static size_t _SensorsStatusImu__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__SensorsStatusImu(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SensorsStatusImu = {
  "px4_msgs::msg",
  "SensorsStatusImu",
  _SensorsStatusImu__cdr_serialize,
  _SensorsStatusImu__cdr_deserialize,
  _SensorsStatusImu__get_serialized_size,
  _SensorsStatusImu__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SensorsStatusImu__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SensorsStatusImu,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__SensorsStatusImu__get_type_hash,
  &px4_msgs__msg__SensorsStatusImu__get_type_description,
  &px4_msgs__msg__SensorsStatusImu__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, SensorsStatusImu)() {
  return &_SensorsStatusImu__type_support;
}

#if defined(__cplusplus)
}
#endif
