// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Event.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/event__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x49, 0x64, 0x81, 0x2e, 0xb9, 0x82, 0xe8, 0xd7,
      0x85, 0x5e, 0x07, 0x63, 0x2a, 0x5b, 0x46, 0x2b,
      0x0b, 0xee, 0xb3, 0xa1, 0x9e, 0x11, 0x69, 0x13,
      0x50, 0x6b, 0x33, 0x9e, 0xb1, 0xc8, 0x76, 0xa9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Event__TYPE_NAME[] = "px4_msgs/msg/Event";

// Define type names, field names, and default values
static char px4_msgs__msg__Event__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Event__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__Event__FIELD_NAME__event_sequence[] = "event_sequence";
static char px4_msgs__msg__Event__FIELD_NAME__arguments[] = "arguments";
static char px4_msgs__msg__Event__FIELD_NAME__log_levels[] = "log_levels";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Event__FIELDS[] = {
  {
    {px4_msgs__msg__Event__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Event__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Event__FIELD_NAME__event_sequence, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Event__FIELD_NAME__arguments, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      25,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Event__FIELD_NAME__log_levels, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Event__TYPE_NAME, 18, 18},
      {px4_msgs__msg__Event__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Events interface\n"
  "uint32 MESSAGE_VERSION = 1\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint32 id                   # Event ID\n"
  "uint16 event_sequence       # Event sequence number\n"
  "uint8[25] arguments         # (optional) arguments, depend on event id\n"
  "\n"
  "uint8 log_levels            # Log levels: 4 bits MSB: internal, 4 bits LSB: external\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 16";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Event__TYPE_NAME, 18, 18},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 385, 385},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Event__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
