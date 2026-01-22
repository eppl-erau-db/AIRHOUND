// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/OpenDroneIdSystem.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/open_drone_id_system__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/open_drone_id_system__struct.h"
#include "px4_msgs/msg/detail/open_drone_id_system__functions.h"
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


using _OpenDroneIdSystem__ros_msg_type = px4_msgs__msg__OpenDroneIdSystem;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__OpenDroneIdSystem(
  const px4_msgs__msg__OpenDroneIdSystem * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: id_or_mac
  {
    size_t size = 20;
    auto array_ptr = ros_message->id_or_mac;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: operator_location_type
  {
    cdr << ros_message->operator_location_type;
  }

  // Field name: classification_type
  {
    cdr << ros_message->classification_type;
  }

  // Field name: operator_latitude
  {
    cdr << ros_message->operator_latitude;
  }

  // Field name: operator_longitude
  {
    cdr << ros_message->operator_longitude;
  }

  // Field name: area_count
  {
    cdr << ros_message->area_count;
  }

  // Field name: area_radius
  {
    cdr << ros_message->area_radius;
  }

  // Field name: area_ceiling
  {
    cdr << ros_message->area_ceiling;
  }

  // Field name: area_floor
  {
    cdr << ros_message->area_floor;
  }

  // Field name: category_eu
  {
    cdr << ros_message->category_eu;
  }

  // Field name: class_eu
  {
    cdr << ros_message->class_eu;
  }

  // Field name: operator_altitude_geo
  {
    cdr << ros_message->operator_altitude_geo;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__OpenDroneIdSystem(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__OpenDroneIdSystem * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: id_or_mac
  {
    size_t size = 20;
    auto array_ptr = ros_message->id_or_mac;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: operator_location_type
  {
    cdr >> ros_message->operator_location_type;
  }

  // Field name: classification_type
  {
    cdr >> ros_message->classification_type;
  }

  // Field name: operator_latitude
  {
    cdr >> ros_message->operator_latitude;
  }

  // Field name: operator_longitude
  {
    cdr >> ros_message->operator_longitude;
  }

  // Field name: area_count
  {
    cdr >> ros_message->area_count;
  }

  // Field name: area_radius
  {
    cdr >> ros_message->area_radius;
  }

  // Field name: area_ceiling
  {
    cdr >> ros_message->area_ceiling;
  }

  // Field name: area_floor
  {
    cdr >> ros_message->area_floor;
  }

  // Field name: category_eu
  {
    cdr >> ros_message->category_eu;
  }

  // Field name: class_eu
  {
    cdr >> ros_message->class_eu;
  }

  // Field name: operator_altitude_geo
  {
    cdr >> ros_message->operator_altitude_geo;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__OpenDroneIdSystem(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _OpenDroneIdSystem__ros_msg_type * ros_message = static_cast<const _OpenDroneIdSystem__ros_msg_type *>(untyped_ros_message);
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

  // Field name: id_or_mac
  {
    size_t array_size = 20;
    auto array_ptr = ros_message->id_or_mac;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_location_type
  {
    size_t item_size = sizeof(ros_message->operator_location_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: classification_type
  {
    size_t item_size = sizeof(ros_message->classification_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_latitude
  {
    size_t item_size = sizeof(ros_message->operator_latitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_longitude
  {
    size_t item_size = sizeof(ros_message->operator_longitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_count
  {
    size_t item_size = sizeof(ros_message->area_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_radius
  {
    size_t item_size = sizeof(ros_message->area_radius);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_ceiling
  {
    size_t item_size = sizeof(ros_message->area_ceiling);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_floor
  {
    size_t item_size = sizeof(ros_message->area_floor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: category_eu
  {
    size_t item_size = sizeof(ros_message->category_eu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: class_eu
  {
    size_t item_size = sizeof(ros_message->class_eu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_altitude_geo
  {
    size_t item_size = sizeof(ros_message->operator_altitude_geo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__OpenDroneIdSystem(
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

  // Field name: id_or_mac
  {
    size_t array_size = 20;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_location_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: classification_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_latitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: operator_longitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: area_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: area_radius
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: area_ceiling
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: area_floor
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: category_eu
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: class_eu
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_altitude_geo
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
    using DataType = px4_msgs__msg__OpenDroneIdSystem;
    is_plain =
      (
      offsetof(DataType, operator_altitude_geo) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__OpenDroneIdSystem(
  const px4_msgs__msg__OpenDroneIdSystem * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: id_or_mac
  {
    size_t size = 20;
    auto array_ptr = ros_message->id_or_mac;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: operator_location_type
  {
    cdr << ros_message->operator_location_type;
  }

  // Field name: classification_type
  {
    cdr << ros_message->classification_type;
  }

  // Field name: operator_latitude
  {
    cdr << ros_message->operator_latitude;
  }

  // Field name: operator_longitude
  {
    cdr << ros_message->operator_longitude;
  }

  // Field name: area_count
  {
    cdr << ros_message->area_count;
  }

  // Field name: area_radius
  {
    cdr << ros_message->area_radius;
  }

  // Field name: area_ceiling
  {
    cdr << ros_message->area_ceiling;
  }

  // Field name: area_floor
  {
    cdr << ros_message->area_floor;
  }

  // Field name: category_eu
  {
    cdr << ros_message->category_eu;
  }

  // Field name: class_eu
  {
    cdr << ros_message->class_eu;
  }

  // Field name: operator_altitude_geo
  {
    cdr << ros_message->operator_altitude_geo;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__OpenDroneIdSystem(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _OpenDroneIdSystem__ros_msg_type * ros_message = static_cast<const _OpenDroneIdSystem__ros_msg_type *>(untyped_ros_message);
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

  // Field name: id_or_mac
  {
    size_t array_size = 20;
    auto array_ptr = ros_message->id_or_mac;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_location_type
  {
    size_t item_size = sizeof(ros_message->operator_location_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: classification_type
  {
    size_t item_size = sizeof(ros_message->classification_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_latitude
  {
    size_t item_size = sizeof(ros_message->operator_latitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_longitude
  {
    size_t item_size = sizeof(ros_message->operator_longitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_count
  {
    size_t item_size = sizeof(ros_message->area_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_radius
  {
    size_t item_size = sizeof(ros_message->area_radius);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_ceiling
  {
    size_t item_size = sizeof(ros_message->area_ceiling);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: area_floor
  {
    size_t item_size = sizeof(ros_message->area_floor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: category_eu
  {
    size_t item_size = sizeof(ros_message->category_eu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: class_eu
  {
    size_t item_size = sizeof(ros_message->class_eu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: operator_altitude_geo
  {
    size_t item_size = sizeof(ros_message->operator_altitude_geo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__OpenDroneIdSystem(
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

  // Field name: id_or_mac
  {
    size_t array_size = 20;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_location_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: classification_type
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_latitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: operator_longitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: area_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: area_radius
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: area_ceiling
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: area_floor
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: category_eu
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: class_eu
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: operator_altitude_geo
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
    using DataType = px4_msgs__msg__OpenDroneIdSystem;
    is_plain =
      (
      offsetof(DataType, operator_altitude_geo) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _OpenDroneIdSystem__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__OpenDroneIdSystem * ros_message = static_cast<const px4_msgs__msg__OpenDroneIdSystem *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__OpenDroneIdSystem(ros_message, cdr);
}

static bool _OpenDroneIdSystem__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__OpenDroneIdSystem * ros_message = static_cast<px4_msgs__msg__OpenDroneIdSystem *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__OpenDroneIdSystem(cdr, ros_message);
}

static uint32_t _OpenDroneIdSystem__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__OpenDroneIdSystem(
      untyped_ros_message, 0));
}

static size_t _OpenDroneIdSystem__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__OpenDroneIdSystem(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_OpenDroneIdSystem = {
  "px4_msgs::msg",
  "OpenDroneIdSystem",
  _OpenDroneIdSystem__cdr_serialize,
  _OpenDroneIdSystem__cdr_deserialize,
  _OpenDroneIdSystem__get_serialized_size,
  _OpenDroneIdSystem__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _OpenDroneIdSystem__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_OpenDroneIdSystem,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__OpenDroneIdSystem__get_type_hash,
  &px4_msgs__msg__OpenDroneIdSystem__get_type_description,
  &px4_msgs__msg__OpenDroneIdSystem__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, OpenDroneIdSystem)() {
  return &_OpenDroneIdSystem__type_support;
}

#if defined(__cplusplus)
}
#endif
