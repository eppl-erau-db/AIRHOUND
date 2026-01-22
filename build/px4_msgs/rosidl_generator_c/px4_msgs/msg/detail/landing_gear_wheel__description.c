// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LandingGearWheel.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/landing_gear_wheel__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LandingGearWheel__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x24, 0xd7, 0x7a, 0xcf, 0x19, 0x62, 0xbb, 0xd6,
      0x9b, 0x7f, 0x16, 0x11, 0xb0, 0xa1, 0x12, 0xad,
      0x44, 0x27, 0x1b, 0x72, 0xf5, 0x1f, 0x49, 0x1d,
      0x7f, 0x80, 0x4f, 0xbc, 0x44, 0x82, 0x6c, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LandingGearWheel__TYPE_NAME[] = "px4_msgs/msg/LandingGearWheel";

// Define type names, field names, and default values
static char px4_msgs__msg__LandingGearWheel__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LandingGearWheel__FIELD_NAME__normalized_wheel_setpoint[] = "normalized_wheel_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LandingGearWheel__FIELDS[] = {
  {
    {px4_msgs__msg__LandingGearWheel__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingGearWheel__FIELD_NAME__normalized_wheel_setpoint, 25, 25},
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
px4_msgs__msg__LandingGearWheel__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LandingGearWheel__TYPE_NAME, 29, 29},
      {px4_msgs__msg__LandingGearWheel__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 normalized_wheel_setpoint\\t# negative is turning left, positive turning right [-1, 1]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LandingGearWheel__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LandingGearWheel__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 152, 152},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LandingGearWheel__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LandingGearWheel__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
