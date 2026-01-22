// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DebugKeyValue.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/debug_key_value__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DebugKeyValue__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0x9a, 0x97, 0x60, 0xf5, 0x05, 0xa9, 0xdf,
      0xed, 0x23, 0xa1, 0x49, 0xd7, 0xee, 0xd3, 0x2d,
      0x03, 0xf8, 0xdd, 0x4f, 0xdd, 0x90, 0xe3, 0xba,
      0x2a, 0xe3, 0x85, 0xe4, 0xf7, 0x7d, 0x3e, 0x63,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DebugKeyValue__TYPE_NAME[] = "px4_msgs/msg/DebugKeyValue";

// Define type names, field names, and default values
static char px4_msgs__msg__DebugKeyValue__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DebugKeyValue__FIELD_NAME__key[] = "key";
static char px4_msgs__msg__DebugKeyValue__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DebugKeyValue__FIELDS[] = {
  {
    {px4_msgs__msg__DebugKeyValue__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugKeyValue__FIELD_NAME__key, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      10,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugKeyValue__FIELD_NAME__value, 5, 5},
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
px4_msgs__msg__DebugKeyValue__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DebugKeyValue__TYPE_NAME, 26, 26},
      {px4_msgs__msg__DebugKeyValue__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "char[10] key\\t\\t\\t# max. 10 characters as key / name\n"
  "float32 value\\t\\t\\t# the value to send as debug output";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DebugKeyValue__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DebugKeyValue__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 161, 161},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DebugKeyValue__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DebugKeyValue__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
