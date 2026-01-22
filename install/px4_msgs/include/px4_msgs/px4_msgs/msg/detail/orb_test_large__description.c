// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OrbTestLarge.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/orb_test_large__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OrbTestLarge__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2f, 0x54, 0xaa, 0x88, 0xbd, 0xea, 0x3c, 0xbd,
      0xec, 0xd6, 0x4e, 0x5f, 0x16, 0x13, 0x23, 0x52,
      0x0e, 0xd5, 0xf3, 0x90, 0xa7, 0xce, 0x19, 0x50,
      0x36, 0xa6, 0xa5, 0xa7, 0xca, 0xca, 0xd2, 0xb3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OrbTestLarge__TYPE_NAME[] = "px4_msgs/msg/OrbTestLarge";

// Define type names, field names, and default values
static char px4_msgs__msg__OrbTestLarge__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OrbTestLarge__FIELD_NAME__val[] = "val";
static char px4_msgs__msg__OrbTestLarge__FIELD_NAME__junk[] = "junk";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OrbTestLarge__FIELDS[] = {
  {
    {px4_msgs__msg__OrbTestLarge__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbTestLarge__FIELD_NAME__val, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbTestLarge__FIELD_NAME__junk, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      512,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OrbTestLarge__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OrbTestLarge__TYPE_NAME, 25, 25},
      {px4_msgs__msg__OrbTestLarge__FIELDS, 3, 3},
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
  "uint8[512] junk";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OrbTestLarge__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OrbTestLarge__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OrbTestLarge__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OrbTestLarge__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
