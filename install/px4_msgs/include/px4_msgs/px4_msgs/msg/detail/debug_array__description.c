// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DebugArray.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/debug_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DebugArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf2, 0xbe, 0xa2, 0x84, 0xb4, 0xf6, 0x3a, 0x18,
      0xa3, 0xb3, 0xff, 0x1d, 0x2b, 0x61, 0xb4, 0x08,
      0x29, 0x81, 0xff, 0x81, 0x34, 0xe8, 0x5c, 0x55,
      0x7c, 0x94, 0x54, 0xa3, 0x01, 0xa6, 0xcb, 0x9a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DebugArray__TYPE_NAME[] = "px4_msgs/msg/DebugArray";

// Define type names, field names, and default values
static char px4_msgs__msg__DebugArray__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DebugArray__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__DebugArray__FIELD_NAME__name[] = "name";
static char px4_msgs__msg__DebugArray__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DebugArray__FIELDS[] = {
  {
    {px4_msgs__msg__DebugArray__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugArray__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugArray__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      10,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DebugArray__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      58,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__DebugArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DebugArray__TYPE_NAME, 23, 23},
      {px4_msgs__msg__DebugArray__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 ARRAY_SIZE = 58\n"
  "uint64 timestamp            # time since system start (microseconds)\n"
  "uint16 id                   # unique ID of debug array, used to discriminate between arrays\n"
  "char[10] name               # name of the debug array (max. 10 characters)\n"
  "float32[58] data            # data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DebugArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DebugArray__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 293, 293},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DebugArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DebugArray__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
