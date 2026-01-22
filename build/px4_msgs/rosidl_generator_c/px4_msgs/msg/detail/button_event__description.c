// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ButtonEvent.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/button_event__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ButtonEvent__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x72, 0x1a, 0xfa, 0xa5, 0x08, 0x51, 0xf8, 0x10,
      0x9e, 0x14, 0x17, 0xda, 0x72, 0xf9, 0xd4, 0xf4,
      0x84, 0x1b, 0x59, 0xc3, 0x1d, 0x86, 0xc3, 0xba,
      0xc8, 0xd2, 0x4c, 0x1b, 0x32, 0x26, 0x9c, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ButtonEvent__TYPE_NAME[] = "px4_msgs/msg/ButtonEvent";

// Define type names, field names, and default values
static char px4_msgs__msg__ButtonEvent__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ButtonEvent__FIELD_NAME__triggered[] = "triggered";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ButtonEvent__FIELDS[] = {
  {
    {px4_msgs__msg__ButtonEvent__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ButtonEvent__FIELD_NAME__triggered, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ButtonEvent__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ButtonEvent__TYPE_NAME, 24, 24},
      {px4_msgs__msg__ButtonEvent__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "bool triggered\\t\\t\\t\\t# Set to true if the event is triggered\n"
  "\n"
  "# TOPICS button_event safety_button\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ButtonEvent__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ButtonEvent__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 183, 183},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ButtonEvent__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ButtonEvent__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
