// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TaskStackInfo.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/task_stack_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TaskStackInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9d, 0x43, 0xf2, 0x31, 0x8b, 0x95, 0x64, 0xfb,
      0x66, 0x00, 0x08, 0x71, 0x65, 0x14, 0xc3, 0xef,
      0x99, 0x50, 0x55, 0x94, 0x09, 0xa3, 0x11, 0x12,
      0x74, 0x1b, 0xc1, 0x73, 0xfb, 0xfa, 0x83, 0xc1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TaskStackInfo__TYPE_NAME[] = "px4_msgs/msg/TaskStackInfo";

// Define type names, field names, and default values
static char px4_msgs__msg__TaskStackInfo__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TaskStackInfo__FIELD_NAME__stack_free[] = "stack_free";
static char px4_msgs__msg__TaskStackInfo__FIELD_NAME__task_name[] = "task_name";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TaskStackInfo__FIELDS[] = {
  {
    {px4_msgs__msg__TaskStackInfo__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TaskStackInfo__FIELD_NAME__stack_free, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TaskStackInfo__FIELD_NAME__task_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      24,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TaskStackInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TaskStackInfo__TYPE_NAME, 26, 26},
      {px4_msgs__msg__TaskStackInfo__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# stack information for a single running process\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint16 stack_free\n"
  "char[24] task_name\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TaskStackInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TaskStackInfo__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 175, 175},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TaskStackInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TaskStackInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
