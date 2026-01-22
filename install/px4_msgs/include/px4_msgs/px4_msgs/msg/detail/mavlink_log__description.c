// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MavlinkLog.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/mavlink_log__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MavlinkLog__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa2, 0x1a, 0x6a, 0x35, 0xe1, 0x24, 0xde, 0x30,
      0x52, 0xac, 0xf6, 0xc6, 0x25, 0x8b, 0x21, 0xf9,
      0xbf, 0x06, 0x47, 0xe6, 0x31, 0x49, 0xdd, 0x55,
      0x9f, 0x59, 0xad, 0xd6, 0x02, 0x67, 0x2e, 0x54,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MavlinkLog__TYPE_NAME[] = "px4_msgs/msg/MavlinkLog";

// Define type names, field names, and default values
static char px4_msgs__msg__MavlinkLog__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MavlinkLog__FIELD_NAME__text[] = "text";
static char px4_msgs__msg__MavlinkLog__FIELD_NAME__severity[] = "severity";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MavlinkLog__FIELDS[] = {
  {
    {px4_msgs__msg__MavlinkLog__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkLog__FIELD_NAME__text, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      127,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MavlinkLog__FIELD_NAME__severity, 8, 8},
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
px4_msgs__msg__MavlinkLog__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MavlinkLog__TYPE_NAME, 23, 23},
      {px4_msgs__msg__MavlinkLog__FIELDS, 3, 3},
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
  "char[127] text\n"
  "uint8 severity # log level (same as in the linux kernel, starting with 0)\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 8";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MavlinkLog__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MavlinkLog__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 177, 177},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MavlinkLog__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MavlinkLog__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
