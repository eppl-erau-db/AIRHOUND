// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ParameterUpdate.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/parameter_update__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ParameterUpdate__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0xe9, 0xbc, 0x28, 0x9a, 0xc4, 0xf5, 0x8f,
      0xe8, 0x4a, 0x25, 0xd5, 0xdd, 0x56, 0x82, 0x7b,
      0x7b, 0x19, 0x62, 0xd4, 0x7a, 0xd4, 0x28, 0x49,
      0x35, 0xa1, 0x8d, 0xba, 0xf3, 0xbc, 0x1a, 0xe5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ParameterUpdate__TYPE_NAME[] = "px4_msgs/msg/ParameterUpdate";

// Define type names, field names, and default values
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__instance[] = "instance";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__get_count[] = "get_count";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__set_count[] = "set_count";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__find_count[] = "find_count";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__export_count[] = "export_count";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__active[] = "active";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__changed[] = "changed";
static char px4_msgs__msg__ParameterUpdate__FIELD_NAME__custom_default[] = "custom_default";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ParameterUpdate__FIELDS[] = {
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__instance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__get_count, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__set_count, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__find_count, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__export_count, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__active, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__changed, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterUpdate__FIELD_NAME__custom_default, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ParameterUpdate__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ParameterUpdate__TYPE_NAME, 28, 28},
      {px4_msgs__msg__ParameterUpdate__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message is used to notify the system about one or more parameter changes\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint32 instance\\t\\t# Instance count - constantly incrementing\n"
  "\n"
  "uint32 get_count\n"
  "uint32 set_count\n"
  "uint32 find_count\n"
  "uint32 export_count\n"
  "\n"
  "uint16 active\n"
  "uint16 changed\n"
  "uint16 custom_default";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ParameterUpdate__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ParameterUpdate__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 326, 326},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ParameterUpdate__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ParameterUpdate__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
