// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Cpuload.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/cpuload__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Cpuload__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf4, 0x13, 0x15, 0xfb, 0x98, 0xba, 0xf7, 0x72,
      0x28, 0xdf, 0x67, 0xb1, 0x59, 0xdf, 0x4f, 0xdd,
      0x97, 0x86, 0x20, 0xd3, 0x8c, 0xd5, 0xf3, 0x8d,
      0xd8, 0x83, 0xd6, 0xfa, 0x38, 0x17, 0xa7, 0xaf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Cpuload__TYPE_NAME[] = "px4_msgs/msg/Cpuload";

// Define type names, field names, and default values
static char px4_msgs__msg__Cpuload__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Cpuload__FIELD_NAME__load[] = "load";
static char px4_msgs__msg__Cpuload__FIELD_NAME__ram_usage[] = "ram_usage";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Cpuload__FIELDS[] = {
  {
    {px4_msgs__msg__Cpuload__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Cpuload__FIELD_NAME__load, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Cpuload__FIELD_NAME__ram_usage, 9, 9},
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
px4_msgs__msg__Cpuload__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Cpuload__TYPE_NAME, 20, 20},
      {px4_msgs__msg__Cpuload__FIELDS, 3, 3},
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
  "float32 load                    # processor load from 0 to 1\n"
  "float32 ram_usage\\t\\t# RAM usage from 0 to 1";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Cpuload__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Cpuload__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 163, 163},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Cpuload__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Cpuload__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
