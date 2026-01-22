// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/WheelEncoders.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/wheel_encoders__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__WheelEncoders__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0x16, 0x56, 0xb0, 0xdc, 0x32, 0x60, 0x93,
      0x2e, 0xa2, 0x58, 0x52, 0x2d, 0x43, 0x92, 0x35,
      0x18, 0x54, 0xda, 0x4b, 0x5c, 0xb1, 0xd6, 0x90,
      0x93, 0xb6, 0xaf, 0xf4, 0x38, 0x73, 0xb9, 0xa9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__WheelEncoders__TYPE_NAME[] = "px4_msgs/msg/WheelEncoders";

// Define type names, field names, and default values
static char px4_msgs__msg__WheelEncoders__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__WheelEncoders__FIELD_NAME__wheel_speed[] = "wheel_speed";
static char px4_msgs__msg__WheelEncoders__FIELD_NAME__wheel_angle[] = "wheel_angle";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__WheelEncoders__FIELDS[] = {
  {
    {px4_msgs__msg__WheelEncoders__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__WheelEncoders__FIELD_NAME__wheel_speed, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__WheelEncoders__FIELD_NAME__wheel_angle, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__WheelEncoders__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__WheelEncoders__TYPE_NAME, 26, 26},
      {px4_msgs__msg__WheelEncoders__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# Two wheels: 0 right, 1 left\n"
  "float32[2] wheel_speed # [rad/s]\n"
  "float32[2] wheel_angle # [rad]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__WheelEncoders__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__WheelEncoders__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 155, 155},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__WheelEncoders__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__WheelEncoders__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
