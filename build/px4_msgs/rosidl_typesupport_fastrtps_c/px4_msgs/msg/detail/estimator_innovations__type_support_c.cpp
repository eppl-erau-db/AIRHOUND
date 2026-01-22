// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/EstimatorInnovations.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_innovations__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/estimator_innovations__struct.h"
#include "px4_msgs/msg/detail/estimator_innovations__functions.h"
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


using _EstimatorInnovations__ros_msg_type = px4_msgs__msg__EstimatorInnovations;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__EstimatorInnovations(
  const px4_msgs__msg__EstimatorInnovations * ros_message,
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

  // Field name: gps_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gps_vvel
  {
    cdr << ros_message->gps_vvel;
  }

  // Field name: gps_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hpos;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gps_vpos
  {
    cdr << ros_message->gps_vpos;
  }

  // Field name: ev_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: ev_vvel
  {
    cdr << ros_message->ev_vvel;
  }

  // Field name: ev_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hpos;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: ev_vpos
  {
    cdr << ros_message->ev_vpos;
  }

  // Field name: rng_vpos
  {
    cdr << ros_message->rng_vpos;
  }

  // Field name: baro_vpos
  {
    cdr << ros_message->baro_vpos;
  }

  // Field name: aux_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->aux_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: flow
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: heading
  {
    cdr << ros_message->heading;
  }

  // Field name: mag_field
  {
    size_t size = 3;
    auto array_ptr = ros_message->mag_field;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gravity
  {
    size_t size = 3;
    auto array_ptr = ros_message->gravity;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: drag
  {
    size_t size = 2;
    auto array_ptr = ros_message->drag;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: airspeed
  {
    cdr << ros_message->airspeed;
  }

  // Field name: beta
  {
    cdr << ros_message->beta;
  }

  // Field name: hagl
  {
    cdr << ros_message->hagl;
  }

  // Field name: hagl_rate
  {
    cdr << ros_message->hagl_rate;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__EstimatorInnovations(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__EstimatorInnovations * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr >> ros_message->timestamp_sample;
  }

  // Field name: gps_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hvel;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gps_vvel
  {
    cdr >> ros_message->gps_vvel;
  }

  // Field name: gps_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hpos;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gps_vpos
  {
    cdr >> ros_message->gps_vpos;
  }

  // Field name: ev_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hvel;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: ev_vvel
  {
    cdr >> ros_message->ev_vvel;
  }

  // Field name: ev_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hpos;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: ev_vpos
  {
    cdr >> ros_message->ev_vpos;
  }

  // Field name: rng_vpos
  {
    cdr >> ros_message->rng_vpos;
  }

  // Field name: baro_vpos
  {
    cdr >> ros_message->baro_vpos;
  }

  // Field name: aux_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->aux_hvel;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: flow
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: heading
  {
    cdr >> ros_message->heading;
  }

  // Field name: mag_field
  {
    size_t size = 3;
    auto array_ptr = ros_message->mag_field;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: gravity
  {
    size_t size = 3;
    auto array_ptr = ros_message->gravity;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: drag
  {
    size_t size = 2;
    auto array_ptr = ros_message->drag;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: airspeed
  {
    cdr >> ros_message->airspeed;
  }

  // Field name: beta
  {
    cdr >> ros_message->beta;
  }

  // Field name: hagl
  {
    cdr >> ros_message->hagl;
  }

  // Field name: hagl_rate
  {
    cdr >> ros_message->hagl_rate;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__EstimatorInnovations(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EstimatorInnovations__ros_msg_type * ros_message = static_cast<const _EstimatorInnovations__ros_msg_type *>(untyped_ros_message);
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

  // Field name: gps_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->gps_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_vvel
  {
    size_t item_size = sizeof(ros_message->gps_vvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_hpos
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->gps_hpos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_vpos
  {
    size_t item_size = sizeof(ros_message->gps_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->ev_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_vvel
  {
    size_t item_size = sizeof(ros_message->ev_vvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_hpos
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->ev_hpos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_vpos
  {
    size_t item_size = sizeof(ros_message->ev_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: rng_vpos
  {
    size_t item_size = sizeof(ros_message->rng_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: baro_vpos
  {
    size_t item_size = sizeof(ros_message->baro_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: aux_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->aux_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: flow
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->flow;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heading
  {
    size_t item_size = sizeof(ros_message->heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: mag_field
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->mag_field;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gravity
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gravity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: drag
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->drag;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: airspeed
  {
    size_t item_size = sizeof(ros_message->airspeed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: beta
  {
    size_t item_size = sizeof(ros_message->beta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hagl
  {
    size_t item_size = sizeof(ros_message->hagl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hagl_rate
  {
    size_t item_size = sizeof(ros_message->hagl_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__EstimatorInnovations(
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

  // Field name: gps_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_vvel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_hpos
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_vvel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_hpos
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: rng_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: baro_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: aux_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: flow
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: heading
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: mag_field
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gravity
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: drag
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: airspeed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: beta
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hagl
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hagl_rate
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
    using DataType = px4_msgs__msg__EstimatorInnovations;
    is_plain =
      (
      offsetof(DataType, hagl_rate) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__EstimatorInnovations(
  const px4_msgs__msg__EstimatorInnovations * ros_message,
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

  // Field name: gps_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gps_vvel
  {
    cdr << ros_message->gps_vvel;
  }

  // Field name: gps_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->gps_hpos;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gps_vpos
  {
    cdr << ros_message->gps_vpos;
  }

  // Field name: ev_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: ev_vvel
  {
    cdr << ros_message->ev_vvel;
  }

  // Field name: ev_hpos
  {
    size_t size = 2;
    auto array_ptr = ros_message->ev_hpos;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: ev_vpos
  {
    cdr << ros_message->ev_vpos;
  }

  // Field name: rng_vpos
  {
    cdr << ros_message->rng_vpos;
  }

  // Field name: baro_vpos
  {
    cdr << ros_message->baro_vpos;
  }

  // Field name: aux_hvel
  {
    size_t size = 2;
    auto array_ptr = ros_message->aux_hvel;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: flow
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: heading
  {
    cdr << ros_message->heading;
  }

  // Field name: mag_field
  {
    size_t size = 3;
    auto array_ptr = ros_message->mag_field;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: gravity
  {
    size_t size = 3;
    auto array_ptr = ros_message->gravity;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: drag
  {
    size_t size = 2;
    auto array_ptr = ros_message->drag;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: airspeed
  {
    cdr << ros_message->airspeed;
  }

  // Field name: beta
  {
    cdr << ros_message->beta;
  }

  // Field name: hagl
  {
    cdr << ros_message->hagl;
  }

  // Field name: hagl_rate
  {
    cdr << ros_message->hagl_rate;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__EstimatorInnovations(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EstimatorInnovations__ros_msg_type * ros_message = static_cast<const _EstimatorInnovations__ros_msg_type *>(untyped_ros_message);
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

  // Field name: gps_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->gps_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_vvel
  {
    size_t item_size = sizeof(ros_message->gps_vvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_hpos
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->gps_hpos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gps_vpos
  {
    size_t item_size = sizeof(ros_message->gps_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->ev_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_vvel
  {
    size_t item_size = sizeof(ros_message->ev_vvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_hpos
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->ev_hpos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ev_vpos
  {
    size_t item_size = sizeof(ros_message->ev_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: rng_vpos
  {
    size_t item_size = sizeof(ros_message->rng_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: baro_vpos
  {
    size_t item_size = sizeof(ros_message->baro_vpos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: aux_hvel
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->aux_hvel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: flow
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->flow;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heading
  {
    size_t item_size = sizeof(ros_message->heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: mag_field
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->mag_field;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: gravity
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gravity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: drag
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->drag;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: airspeed
  {
    size_t item_size = sizeof(ros_message->airspeed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: beta
  {
    size_t item_size = sizeof(ros_message->beta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hagl
  {
    size_t item_size = sizeof(ros_message->hagl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hagl_rate
  {
    size_t item_size = sizeof(ros_message->hagl_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__EstimatorInnovations(
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

  // Field name: gps_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_vvel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_hpos
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gps_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_vvel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_hpos
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ev_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: rng_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: baro_vpos
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: aux_hvel
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: flow
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: heading
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: mag_field
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: gravity
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: drag
  {
    size_t array_size = 2;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: airspeed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: beta
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hagl
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hagl_rate
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
    using DataType = px4_msgs__msg__EstimatorInnovations;
    is_plain =
      (
      offsetof(DataType, hagl_rate) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _EstimatorInnovations__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__EstimatorInnovations * ros_message = static_cast<const px4_msgs__msg__EstimatorInnovations *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__EstimatorInnovations(ros_message, cdr);
}

static bool _EstimatorInnovations__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__EstimatorInnovations * ros_message = static_cast<px4_msgs__msg__EstimatorInnovations *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__EstimatorInnovations(cdr, ros_message);
}

static uint32_t _EstimatorInnovations__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__EstimatorInnovations(
      untyped_ros_message, 0));
}

static size_t _EstimatorInnovations__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__EstimatorInnovations(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EstimatorInnovations = {
  "px4_msgs::msg",
  "EstimatorInnovations",
  _EstimatorInnovations__cdr_serialize,
  _EstimatorInnovations__cdr_deserialize,
  _EstimatorInnovations__get_serialized_size,
  _EstimatorInnovations__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _EstimatorInnovations__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EstimatorInnovations,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__EstimatorInnovations__get_type_hash,
  &px4_msgs__msg__EstimatorInnovations__get_type_description,
  &px4_msgs__msg__EstimatorInnovations__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, EstimatorInnovations)() {
  return &_EstimatorInnovations__type_support;
}

#if defined(__cplusplus)
}
#endif
