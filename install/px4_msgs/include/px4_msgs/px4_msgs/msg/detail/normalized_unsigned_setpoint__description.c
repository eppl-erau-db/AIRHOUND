// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/NormalizedUnsignedSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/normalized_unsigned_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__NormalizedUnsignedSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x89, 0x6e, 0xe4, 0xe4, 0xd3, 0x1b, 0x7c, 0x02,
      0xcf, 0xe3, 0x9c, 0x76, 0x0c, 0x59, 0xe8, 0x9c,
      0x45, 0x81, 0x76, 0xf7, 0xe4, 0x15, 0x23, 0x21,
      0x78, 0x36, 0xe3, 0x6d, 0x9a, 0x9d, 0xa1, 0x87,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__NormalizedUnsignedSetpoint__TYPE_NAME[] = "px4_msgs/msg/NormalizedUnsignedSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__NormalizedUnsignedSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__NormalizedUnsignedSetpoint__FIELD_NAME__normalized_setpoint[] = "normalized_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__NormalizedUnsignedSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__NormalizedUnsignedSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NormalizedUnsignedSetpoint__FIELD_NAME__normalized_setpoint, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__NormalizedUnsignedSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__NormalizedUnsignedSetpoint__TYPE_NAME, 39, 39},
      {px4_msgs__msg__NormalizedUnsignedSetpoint__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp        \\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32 normalized_setpoint          \\t# [0, 1]\n"
  "\n"
  "# TOPICS flaps_setpoint spoilers_setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__NormalizedUnsignedSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__NormalizedUnsignedSetpoint__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 158, 158},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__NormalizedUnsignedSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__NormalizedUnsignedSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
