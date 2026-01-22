// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/ObstacleDistance.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/obstacle_distance__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/obstacle_distance__struct.h"
#include "px4_msgs/msg/detail/obstacle_distance__functions.h"
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


using _ObstacleDistance__ros_msg_type = px4_msgs__msg__ObstacleDistance;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__ObstacleDistance(
  const px4_msgs__msg__ObstacleDistance * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: frame
  {
    cdr << ros_message->frame;
  }

  // Field name: sensor_type
  {
    cdr << ros_message->sensor_type;
  }

  // Field name: distances
  {
    size_t size = 72;
    auto array_ptr = ros_message->distances;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: increment
  {
    cdr << ros_message->increment;
  }

  // Field name: min_distance
  {
    cdr << ros_message->min_distance;
  }

  // Field name: max_distance
  {
    cdr << ros_message->max_distance;
  }

  // Field name: angle_offset
  {
    cdr << ros_message->angle_offset;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__ObstacleDistance(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__ObstacleDistance * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: frame
  {
    cdr >> ros_message->frame;
  }

  // Field name: sensor_type
  {
    cdr >> ros_message->sensor_type;
  }

  // Field name: distances
  {
    size_t size = 72;
    auto array_ptr = ros_message->distances;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: increment
  {
    cdr >> ros_message->increment;
  }

  // Field name: min_distance
  {
    cdr >> ros_message->min_distance;
  }

  // Field name: max_distance
  {
    cdr >> ros_message->max_distance;
  }

  // Field name: angle_offset
  {
    cdr >> ros_message->angle_offset;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__ObstacleDistance(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ObstacleDistance__ros_msg_type * ros_message = static_cast<const _ObstacleDistance__ros_msg_type *>(untyped_ros_message);
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

  // Field name: frame
  {
    size_t item_size = sizeof(ros_message->frame);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sensor_type
  {
    size_t item_size = sizeof(ros_message->sensor_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: distances
  {
    size_t array_size = 72;
    auto array_ptr = ros_message->distances;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: increment
  {
    size_t item_size = sizeof(ros_message->increment);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: min_distance
  {
    size_t item_size = sizeof(ros_message->min_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_distance
  {
    size_t item_size = sizeof(ros_message->max_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: angle_offset
  {
    size_t item_size = sizeof(ros_message->angle_offset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__ObstacleDistance(
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

  // Field name: frame
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: sensor_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: distances
  {
    size_t array_size = 72;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: increment
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: min_distance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: max_distance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: angle_offset
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__ObstacleDistance;
    is_plain =
      (
      offsetof(DataType, angle_offset) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__ObstacleDistance(
  const px4_msgs__msg__ObstacleDistance * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: frame
  {
    cdr << ros_message->frame;
  }

  // Field name: sensor_type
  {
    cdr << ros_message->sensor_type;
  }

  // Field name: distances
  {
    size_t size = 72;
    auto array_ptr = ros_message->distances;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: increment
  {
    cdr << ros_message->increment;
  }

  // Field name: min_distance
  {
    cdr << ros_message->min_distance;
  }

  // Field name: max_distance
  {
    cdr << ros_message->max_distance;
  }

  // Field name: angle_offset
  {
    cdr << ros_message->angle_offset;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__ObstacleDistance(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ObstacleDistance__ros_msg_type * ros_message = static_cast<const _ObstacleDistance__ros_msg_type *>(untyped_ros_message);
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

  // Field name: frame
  {
    size_t item_size = sizeof(ros_message->frame);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sensor_type
  {
    size_t item_size = sizeof(ros_message->sensor_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: distances
  {
    size_t array_size = 72;
    auto array_ptr = ros_message->distances;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: increment
  {
    size_t item_size = sizeof(ros_message->increment);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: min_distance
  {
    size_t item_size = sizeof(ros_message->min_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_distance
  {
    size_t item_size = sizeof(ros_message->max_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: angle_offset
  {
    size_t item_size = sizeof(ros_message->angle_offset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__ObstacleDistance(
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

  // Field name: frame
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: sensor_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: distances
  {
    size_t array_size = 72;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: increment
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: min_distance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: max_distance
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: angle_offset
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__ObstacleDistance;
    is_plain =
      (
      offsetof(DataType, angle_offset) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ObstacleDistance__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__ObstacleDistance * ros_message = static_cast<const px4_msgs__msg__ObstacleDistance *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__ObstacleDistance(ros_message, cdr);
}

static bool _ObstacleDistance__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__ObstacleDistance * ros_message = static_cast<px4_msgs__msg__ObstacleDistance *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__ObstacleDistance(cdr, ros_message);
}

static uint32_t _ObstacleDistance__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__ObstacleDistance(
      untyped_ros_message, 0));
}

static size_t _ObstacleDistance__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__ObstacleDistance(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ObstacleDistance = {
  "px4_msgs::msg",
  "ObstacleDistance",
  _ObstacleDistance__cdr_serialize,
  _ObstacleDistance__cdr_deserialize,
  _ObstacleDistance__get_serialized_size,
  _ObstacleDistance__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ObstacleDistance__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ObstacleDistance,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__ObstacleDistance__get_type_hash,
  &px4_msgs__msg__ObstacleDistance__get_type_description,
  &px4_msgs__msg__ObstacleDistance__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, ObstacleDistance)() {
  return &_ObstacleDistance__type_support;
}

#if defined(__cplusplus)
}
#endif
