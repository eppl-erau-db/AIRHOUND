// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/debug_value__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DebugValue__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2c, 0xd7, 0xb6, 0x70, 0xfa, 0x96, 0x1c, 0x82,
      0xc4, 0x15, 0xe0, 0xce, 0xcd, 0xe4, 0x22, 0xbf,
      0x27, 0xbf, 0xcf, 0xff, 0x94, 0xe7, 0x78, 0x4e,
      0x80, 0xa1, 0x96, 0x1f, 0x57, 0x50, 0xa2, 0x1d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DebugValue__TYPE_NAME[] = "px4_msgs/msg/DebugValue";

// Define type names, field names, and default values
static char px4_msgs__msg__DebugValue__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DebugValue__FIELD_NAME__ind[] = "ind";
static char px4_msgs__msg__DebugValue__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DebugValue__FIELDS[] = {
  {
    {px4_msgs__msg__DebugValue__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugValue__FIELD_NAME__ind, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugValue__FIELD_NAME__value, 5, 5},
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
px4_msgs__msg__DebugValue__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DebugValue__TYPE_NAME, 23, 23},
      {px4_msgs__msg__DebugValue__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "int8 ind                # index of debug variable\n"
  "float32 value           # the value to send as debug output";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DebugValue__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DebugValue__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 168, 168},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DebugValue__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DebugValue__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
