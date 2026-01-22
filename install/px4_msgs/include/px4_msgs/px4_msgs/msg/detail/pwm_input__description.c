// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PwmInput.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/pwm_input__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PwmInput__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x78, 0x5a, 0x7a, 0xa7, 0xc5, 0x6b, 0x30, 0x8f,
      0x4c, 0x20, 0x21, 0x32, 0x19, 0x29, 0x57, 0x87,
      0x8b, 0xbe, 0x1f, 0x6c, 0xa5, 0xd7, 0x83, 0x64,
      0x23, 0xbd, 0xb2, 0x5f, 0x63, 0xbe, 0xd8, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__PwmInput__TYPE_NAME[] = "px4_msgs/msg/PwmInput";

// Define type names, field names, and default values
static char px4_msgs__msg__PwmInput__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PwmInput__FIELD_NAME__error_count[] = "error_count";
static char px4_msgs__msg__PwmInput__FIELD_NAME__pulse_width[] = "pulse_width";
static char px4_msgs__msg__PwmInput__FIELD_NAME__period[] = "period";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PwmInput__FIELDS[] = {
  {
    {px4_msgs__msg__PwmInput__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PwmInput__FIELD_NAME__error_count, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PwmInput__FIELD_NAME__pulse_width, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PwmInput__FIELD_NAME__period, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__PwmInput__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PwmInput__TYPE_NAME, 21, 21},
      {px4_msgs__msg__PwmInput__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp   # Time since system start (microseconds)\n"
  "uint64 error_count # Timer overcapture error flag (AUX5 or MAIN5)\n"
  "uint32 pulse_width # Pulse width, timer counts (microseconds)\n"
  "uint32 period      # Period, timer counts (microseconds)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PwmInput__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PwmInput__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 245, 245},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PwmInput__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PwmInput__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
