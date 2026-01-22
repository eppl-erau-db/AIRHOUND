// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OrbTest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/orb_test__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OrbTest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0xf9, 0xf4, 0x53, 0xc5, 0xfa, 0x93, 0xb0,
      0x10, 0xae, 0xa8, 0x28, 0x0d, 0xf4, 0xe0, 0xd6,
      0xa0, 0xb3, 0x67, 0xac, 0x9d, 0xdf, 0x49, 0x9c,
      0x44, 0xd8, 0x22, 0x2d, 0x78, 0xe7, 0x83, 0x18,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OrbTest__TYPE_NAME[] = "px4_msgs/msg/OrbTest";

// Define type names, field names, and default values
static char px4_msgs__msg__OrbTest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OrbTest__FIELD_NAME__val[] = "val";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OrbTest__FIELDS[] = {
  {
    {px4_msgs__msg__OrbTest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbTest__FIELD_NAME__val, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OrbTest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OrbTest__TYPE_NAME, 20, 20},
      {px4_msgs__msg__OrbTest__FIELDS, 2, 2},
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
  "\n"
  "int32 val\n"
  "\n"
  "# TOPICS orb_test orb_multitest";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OrbTest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OrbTest__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 103, 103},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OrbTest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OrbTest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
