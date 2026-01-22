// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/YawEstimatorStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/yaw_estimator_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/yaw_estimator_status__struct.h"
#include "px4_msgs/msg/detail/yaw_estimator_status__functions.h"
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


using _YawEstimatorStatus__ros_msg_type = px4_msgs__msg__YawEstimatorStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__YawEstimatorStatus(
  const px4_msgs__msg__YawEstimatorStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr << ros_message->timestamp_sample;
  }

  // Field name: yaw_composite
  {
    cdr << ros_message->yaw_composite;
  }

  // Field name: yaw_variance
  {
    cdr << ros_message->yaw_variance;
  }

  // Field name: yaw_composite_valid
  {
    cdr << (ros_message->yaw_composite_valid ? true : false);
  }

  // Field name: yaw
  {
    size_t size = 5;
    auto array_ptr = ros_message->yaw;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: innov_vn
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_vn;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: innov_ve
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_ve;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    size_t size = 5;
    auto array_ptr = ros_message->weight;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__YawEstimatorStatus(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__YawEstimatorStatus * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr >> ros_message->timestamp_sample;
  }

  // Field name: yaw_composite
  {
    cdr >> ros_message->yaw_composite;
  }

  // Field name: yaw_variance
  {
    cdr >> ros_message->yaw_variance;
  }

  // Field name: yaw_composite_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->yaw_composite_valid = tmp ? true : false;
  }

  // Field name: yaw
  {
    size_t size = 5;
    auto array_ptr = ros_message->yaw;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: innov_vn
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_vn;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: innov_ve
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_ve;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    size_t size = 5;
    auto array_ptr = ros_message->weight;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__YawEstimatorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _YawEstimatorStatus__ros_msg_type * ros_message = static_cast<const _YawEstimatorStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: timestamp_sample
  {
    size_t item_size = sizeof(ros_message->timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_composite
  {
    size_t item_size = sizeof(ros_message->yaw_composite);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_variance
  {
    size_t item_size = sizeof(ros_message->yaw_variance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_composite_valid
  {
    size_t item_size = sizeof(ros_message->yaw_composite_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->yaw;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: innov_vn
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->innov_vn;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: innov_ve
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->innov_ve;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: weight
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->weight;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__YawEstimatorStatus(
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

  // Field name: timestamp_sample
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: yaw_composite
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_variance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_composite_valid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: yaw
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: innov_vn
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: innov_ve
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: weight
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__YawEstimatorStatus;
    is_plain =
      (
      offsetof(DataType, weight) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__YawEstimatorStatus(
  const px4_msgs__msg__YawEstimatorStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr << ros_message->timestamp_sample;
  }

  // Field name: yaw_composite
  {
    cdr << ros_message->yaw_composite;
  }

  // Field name: yaw_variance
  {
    cdr << ros_message->yaw_variance;
  }

  // Field name: yaw_composite_valid
  {
    cdr << (ros_message->yaw_composite_valid ? true : false);
  }

  // Field name: yaw
  {
    size_t size = 5;
    auto array_ptr = ros_message->yaw;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: innov_vn
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_vn;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: innov_ve
  {
    size_t size = 5;
    auto array_ptr = ros_message->innov_ve;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    size_t size = 5;
    auto array_ptr = ros_message->weight;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__YawEstimatorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _YawEstimatorStatus__ros_msg_type * ros_message = static_cast<const _YawEstimatorStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: timestamp_sample
  {
    size_t item_size = sizeof(ros_message->timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_composite
  {
    size_t item_size = sizeof(ros_message->yaw_composite);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_variance
  {
    size_t item_size = sizeof(ros_message->yaw_variance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_composite_valid
  {
    size_t item_size = sizeof(ros_message->yaw_composite_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->yaw;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: innov_vn
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->innov_vn;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: innov_ve
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->innov_ve;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: weight
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->weight;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__YawEstimatorStatus(
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

  // Field name: timestamp_sample
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: yaw_composite
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_variance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: yaw_composite_valid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: yaw
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: innov_vn
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: innov_ve
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: weight
  {
    size_t array_size = 5;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__YawEstimatorStatus;
    is_plain =
      (
      offsetof(DataType, weight) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _YawEstimatorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__YawEstimatorStatus * ros_message = static_cast<const px4_msgs__msg__YawEstimatorStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__YawEstimatorStatus(ros_message, cdr);
}

static bool _YawEstimatorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__YawEstimatorStatus * ros_message = static_cast<px4_msgs__msg__YawEstimatorStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__YawEstimatorStatus(cdr, ros_message);
}

static uint32_t _YawEstimatorStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__YawEstimatorStatus(
      untyped_ros_message, 0));
}

static size_t _YawEstimatorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__YawEstimatorStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_YawEstimatorStatus = {
  "px4_msgs::msg",
  "YawEstimatorStatus",
  _YawEstimatorStatus__cdr_serialize,
  _YawEstimatorStatus__cdr_deserialize,
  _YawEstimatorStatus__get_serialized_size,
  _YawEstimatorStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _YawEstimatorStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_YawEstimatorStatus,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__YawEstimatorStatus__get_type_hash,
  &px4_msgs__msg__YawEstimatorStatus__get_type_description,
  &px4_msgs__msg__YawEstimatorStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, YawEstimatorStatus)() {
  return &_YawEstimatorStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
