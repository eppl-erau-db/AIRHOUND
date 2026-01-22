// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/UnregisterExtComponent.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/unregister_ext_component__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__UnregisterExtComponent__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x93, 0xb6, 0x73, 0xec, 0x97, 0x2c, 0xd7, 0xee,
      0x9d, 0xfb, 0x32, 0x5f, 0x4d, 0xd7, 0xdb, 0xf7,
      0xb3, 0xeb, 0x99, 0x0a, 0xf9, 0x4b, 0x3a, 0x64,
      0x67, 0xfc, 0x92, 0x78, 0x82, 0x7b, 0xa0, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__UnregisterExtComponent__TYPE_NAME[] = "px4_msgs/msg/UnregisterExtComponent";

// Define type names, field names, and default values
static char px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__name[] = "name";
static char px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__arming_check_id[] = "arming_check_id";
static char px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__mode_id[] = "mode_id";
static char px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__mode_executor_id[] = "mode_executor_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__UnregisterExtComponent__FIELDS[] = {
  {
    {px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      25,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__arming_check_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__mode_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UnregisterExtComponent__FIELD_NAME__mode_executor_id, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__UnregisterExtComponent__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__UnregisterExtComponent__TYPE_NAME, 35, 35},
      {px4_msgs__msg__UnregisterExtComponent__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "char[25] name                      # either the mode name, or component name\n"
  "\n"
  "int8 arming_check_id      # arming check registration ID (-1 if not registered)\n"
  "int8 mode_id              # assigned mode ID (-1 if not registered)\n"
  "int8 mode_executor_id     # assigned mode executor ID (-1 if not registered)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__UnregisterExtComponent__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__UnregisterExtComponent__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 390, 390},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__UnregisterExtComponent__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__UnregisterExtComponent__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
