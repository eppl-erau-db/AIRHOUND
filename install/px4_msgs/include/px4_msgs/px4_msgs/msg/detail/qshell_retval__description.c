// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/QshellRetval.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/qshell_retval__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__QshellRetval__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9f, 0x9b, 0xd2, 0xd3, 0xac, 0x3b, 0x79, 0x59,
      0xe2, 0x95, 0x27, 0x97, 0xcb, 0x78, 0x34, 0xb7,
      0xa8, 0xee, 0x72, 0xb5, 0x70, 0xa6, 0x5e, 0x92,
      0xed, 0xb2, 0x1d, 0x8a, 0x65, 0x9a, 0x81, 0xc4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__QshellRetval__TYPE_NAME[] = "px4_msgs/msg/QshellRetval";

// Define type names, field names, and default values
static char px4_msgs__msg__QshellRetval__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__QshellRetval__FIELD_NAME__return_value[] = "return_value";
static char px4_msgs__msg__QshellRetval__FIELD_NAME__return_sequence[] = "return_sequence";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__QshellRetval__FIELDS[] = {
  {
    {px4_msgs__msg__QshellRetval__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__QshellRetval__FIELD_NAME__return_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__QshellRetval__FIELD_NAME__return_sequence, 15, 15},
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
px4_msgs__msg__QshellRetval__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__QshellRetval__TYPE_NAME, 25, 25},
      {px4_msgs__msg__QshellRetval__FIELDS, 3, 3},
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
  "int32 return_value\n"
  "uint32 return_sequence";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__QshellRetval__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__QshellRetval__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 101, 101},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__QshellRetval__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__QshellRetval__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
