// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LogMessage.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/log_message__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LogMessage__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x98, 0x1d, 0xcd, 0xbe, 0xc9, 0x6a, 0x04,
      0x3c, 0x42, 0x22, 0xed, 0x9f, 0x3e, 0xcd, 0x48,
      0x82, 0x5c, 0x6d, 0x7a, 0x90, 0xbc, 0x28, 0x86,
      0xf0, 0x13, 0xf2, 0x1d, 0xbc, 0x71, 0x94, 0xf2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LogMessage__TYPE_NAME[] = "px4_msgs/msg/LogMessage";

// Define type names, field names, and default values
static char px4_msgs__msg__LogMessage__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LogMessage__FIELD_NAME__severity[] = "severity";
static char px4_msgs__msg__LogMessage__FIELD_NAME__text[] = "text";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LogMessage__FIELDS[] = {
  {
    {px4_msgs__msg__LogMessage__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LogMessage__FIELD_NAME__severity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LogMessage__FIELD_NAME__text, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      127,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__LogMessage__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LogMessage__TYPE_NAME, 23, 23},
      {px4_msgs__msg__LogMessage__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A logging message, output with PX4_WARN, PX4_ERR, PX4_INFO\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 severity # log level (same as in the linux kernel, starting with 0)\n"
  "char[127] text\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LogMessage__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LogMessage__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 239, 239},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LogMessage__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LogMessage__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
