// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/EscStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/esc_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/esc_status__struct.h"
#include "px4_msgs/msg/detail/esc_status__functions.h"
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

#include "px4_msgs/msg/detail/esc_report__functions.h"  // esc

// forward declare type support functions

bool cdr_serialize_px4_msgs__msg__EscReport(
  const px4_msgs__msg__EscReport * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_px4_msgs__msg__EscReport(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__EscReport * ros_message);

size_t get_serialized_size_px4_msgs__msg__EscReport(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_px4_msgs__msg__EscReport(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_px4_msgs__msg__EscReport(
  const px4_msgs__msg__EscReport * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_px4_msgs__msg__EscReport(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_px4_msgs__msg__EscReport(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, EscReport)();


using _EscStatus__ros_msg_type = px4_msgs__msg__EscStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__EscStatus(
  const px4_msgs__msg__EscStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: counter
  {
    cdr << ros_message->counter;
  }

  // Field name: esc_count
  {
    cdr << ros_message->esc_count;
  }

  // Field name: esc_connectiontype
  {
    cdr << ros_message->esc_connectiontype;
  }

  // Field name: esc_online_flags
  {
    cdr << ros_message->esc_online_flags;
  }

  // Field name: esc_armed_flags
  {
    cdr << ros_message->esc_armed_flags;
  }

  // Field name: esc
  {
    size_t size = 8;
    auto array_ptr = ros_message->esc;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_px4_msgs__msg__EscReport(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__EscStatus(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__EscStatus * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: counter
  {
    cdr >> ros_message->counter;
  }

  // Field name: esc_count
  {
    cdr >> ros_message->esc_count;
  }

  // Field name: esc_connectiontype
  {
    cdr >> ros_message->esc_connectiontype;
  }

  // Field name: esc_online_flags
  {
    cdr >> ros_message->esc_online_flags;
  }

  // Field name: esc_armed_flags
  {
    cdr >> ros_message->esc_armed_flags;
  }

  // Field name: esc
  {
    size_t size = 8;
    auto array_ptr = ros_message->esc;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_px4_msgs__msg__EscReport(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__EscStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EscStatus__ros_msg_type * ros_message = static_cast<const _EscStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: counter
  {
    size_t item_size = sizeof(ros_message->counter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_count
  {
    size_t item_size = sizeof(ros_message->esc_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_connectiontype
  {
    size_t item_size = sizeof(ros_message->esc_connectiontype);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_online_flags
  {
    size_t item_size = sizeof(ros_message->esc_online_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_armed_flags
  {
    size_t item_size = sizeof(ros_message->esc_armed_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc
  {
    size_t array_size = 8;
    auto array_ptr = ros_message->esc;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_px4_msgs__msg__EscReport(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__EscStatus(
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

  // Field name: counter
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: esc_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_connectiontype
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_online_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_armed_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc
  {
    size_t array_size = 8;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_px4_msgs__msg__EscReport(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__EscStatus;
    is_plain =
      (
      offsetof(DataType, esc) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__EscStatus(
  const px4_msgs__msg__EscStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: counter
  {
    cdr << ros_message->counter;
  }

  // Field name: esc_count
  {
    cdr << ros_message->esc_count;
  }

  // Field name: esc_connectiontype
  {
    cdr << ros_message->esc_connectiontype;
  }

  // Field name: esc_online_flags
  {
    cdr << ros_message->esc_online_flags;
  }

  // Field name: esc_armed_flags
  {
    cdr << ros_message->esc_armed_flags;
  }

  // Field name: esc
  {
    size_t size = 8;
    auto array_ptr = ros_message->esc;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_px4_msgs__msg__EscReport(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__EscStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EscStatus__ros_msg_type * ros_message = static_cast<const _EscStatus__ros_msg_type *>(untyped_ros_message);
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

  // Field name: counter
  {
    size_t item_size = sizeof(ros_message->counter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_count
  {
    size_t item_size = sizeof(ros_message->esc_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_connectiontype
  {
    size_t item_size = sizeof(ros_message->esc_connectiontype);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_online_flags
  {
    size_t item_size = sizeof(ros_message->esc_online_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc_armed_flags
  {
    size_t item_size = sizeof(ros_message->esc_armed_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: esc
  {
    size_t array_size = 8;
    auto array_ptr = ros_message->esc;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_px4_msgs__msg__EscReport(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__EscStatus(
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

  // Field name: counter
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: esc_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_connectiontype
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_online_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc_armed_flags
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: esc
  {
    size_t array_size = 8;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_px4_msgs__msg__EscReport(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__EscStatus;
    is_plain =
      (
      offsetof(DataType, esc) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _EscStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__EscStatus * ros_message = static_cast<const px4_msgs__msg__EscStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__EscStatus(ros_message, cdr);
}

static bool _EscStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__EscStatus * ros_message = static_cast<px4_msgs__msg__EscStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__EscStatus(cdr, ros_message);
}

static uint32_t _EscStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__EscStatus(
      untyped_ros_message, 0));
}

static size_t _EscStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__EscStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EscStatus = {
  "px4_msgs::msg",
  "EscStatus",
  _EscStatus__cdr_serialize,
  _EscStatus__cdr_deserialize,
  _EscStatus__get_serialized_size,
  _EscStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _EscStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EscStatus,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__EscStatus__get_type_hash,
  &px4_msgs__msg__EscStatus__get_type_description,
  &px4_msgs__msg__EscStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, EscStatus)() {
  return &_EscStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
