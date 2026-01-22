// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/RcParameterMap.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rc_parameter_map__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/rc_parameter_map__struct.h"
#include "px4_msgs/msg/detail/rc_parameter_map__functions.h"
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


using _RcParameterMap__ros_msg_type = px4_msgs__msg__RcParameterMap;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__RcParameterMap(
  const px4_msgs__msg__RcParameterMap * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: valid
  {
    size_t size = 3;
    auto array_ptr = ros_message->valid;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: param_index
  {
    size_t size = 3;
    auto array_ptr = ros_message->param_index;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: param_id
  {
    size_t size = 51;
    auto array_ptr = ros_message->param_id;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: scale
  {
    size_t size = 3;
    auto array_ptr = ros_message->scale;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value0
  {
    size_t size = 3;
    auto array_ptr = ros_message->value0;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value_min
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_min;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value_max
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_max;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__RcParameterMap(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__RcParameterMap * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: valid
  {
    size_t size = 3;
    auto array_ptr = ros_message->valid;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: param_index
  {
    size_t size = 3;
    auto array_ptr = ros_message->param_index;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: param_id
  {
    size_t size = 51;
    auto array_ptr = ros_message->param_id;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: scale
  {
    size_t size = 3;
    auto array_ptr = ros_message->scale;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: value0
  {
    size_t size = 3;
    auto array_ptr = ros_message->value0;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: value_min
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_min;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: value_max
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_max;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__RcParameterMap(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RcParameterMap__ros_msg_type * ros_message = static_cast<const _RcParameterMap__ros_msg_type *>(untyped_ros_message);
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

  // Field name: valid
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->valid;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: param_index
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->param_index;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: param_id
  {
    size_t array_size = 51;
    auto array_ptr = ros_message->param_id;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: scale
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->scale;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value0
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value0;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value_min
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value_min;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value_max
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value_max;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__RcParameterMap(
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

  // Field name: valid
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: param_index
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: param_id
  {
    size_t array_size = 51;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: scale
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value0
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value_min
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value_max
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__RcParameterMap;
    is_plain =
      (
      offsetof(DataType, value_max) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__RcParameterMap(
  const px4_msgs__msg__RcParameterMap * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: valid
  {
    size_t size = 3;
    auto array_ptr = ros_message->valid;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: param_index
  {
    size_t size = 3;
    auto array_ptr = ros_message->param_index;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: param_id
  {
    size_t size = 51;
    auto array_ptr = ros_message->param_id;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: scale
  {
    size_t size = 3;
    auto array_ptr = ros_message->scale;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value0
  {
    size_t size = 3;
    auto array_ptr = ros_message->value0;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value_min
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_min;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: value_max
  {
    size_t size = 3;
    auto array_ptr = ros_message->value_max;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__RcParameterMap(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RcParameterMap__ros_msg_type * ros_message = static_cast<const _RcParameterMap__ros_msg_type *>(untyped_ros_message);
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

  // Field name: valid
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->valid;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: param_index
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->param_index;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: param_id
  {
    size_t array_size = 51;
    auto array_ptr = ros_message->param_id;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: scale
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->scale;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value0
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value0;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value_min
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value_min;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: value_max
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->value_max;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__RcParameterMap(
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

  // Field name: valid
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: param_index
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: param_id
  {
    size_t array_size = 51;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: scale
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value0
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value_min
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: value_max
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__RcParameterMap;
    is_plain =
      (
      offsetof(DataType, value_max) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _RcParameterMap__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__RcParameterMap * ros_message = static_cast<const px4_msgs__msg__RcParameterMap *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__RcParameterMap(ros_message, cdr);
}

static bool _RcParameterMap__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__RcParameterMap * ros_message = static_cast<px4_msgs__msg__RcParameterMap *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__RcParameterMap(cdr, ros_message);
}

static uint32_t _RcParameterMap__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__RcParameterMap(
      untyped_ros_message, 0));
}

static size_t _RcParameterMap__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__RcParameterMap(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RcParameterMap = {
  "px4_msgs::msg",
  "RcParameterMap",
  _RcParameterMap__cdr_serialize,
  _RcParameterMap__cdr_deserialize,
  _RcParameterMap__get_serialized_size,
  _RcParameterMap__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RcParameterMap__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RcParameterMap,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__RcParameterMap__get_type_hash,
  &px4_msgs__msg__RcParameterMap__get_type_description,
  &px4_msgs__msg__RcParameterMap__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, RcParameterMap)() {
  return &_RcParameterMap__type_support;
}

#if defined(__cplusplus)
}
#endif
