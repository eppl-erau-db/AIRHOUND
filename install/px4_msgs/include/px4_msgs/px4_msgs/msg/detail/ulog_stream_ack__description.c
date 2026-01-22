// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/UlogStreamAck.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/ulog_stream_ack__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__UlogStreamAck__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0x11, 0xd6, 0x1a, 0x79, 0x6f, 0x77, 0xec,
      0xa3, 0x24, 0x36, 0x9b, 0xf1, 0xc0, 0x16, 0x05,
      0xa7, 0x63, 0x04, 0x7a, 0xec, 0x8f, 0x94, 0x0f,
      0xe7, 0x92, 0x64, 0x21, 0x79, 0x87, 0x46, 0x9d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__UlogStreamAck__TYPE_NAME[] = "px4_msgs/msg/UlogStreamAck";

// Define type names, field names, and default values
static char px4_msgs__msg__UlogStreamAck__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__UlogStreamAck__FIELD_NAME__msg_sequence[] = "msg_sequence";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__UlogStreamAck__FIELDS[] = {
  {
    {px4_msgs__msg__UlogStreamAck__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStreamAck__FIELD_NAME__msg_sequence, 12, 12},
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
px4_msgs__msg__UlogStreamAck__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__UlogStreamAck__TYPE_NAME, 26, 26},
      {px4_msgs__msg__UlogStreamAck__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Ack a previously sent ulog_stream message that had\n"
  "# the NEED_ACK flag set\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "int32 ACK_TIMEOUT = 50\\t\\t# timeout waiting for an ack until we retry to send the message [ms]\n"
  "int32 ACK_MAX_TRIES = 50\\t# maximum amount of tries to (re-)send a message, each time waiting ACK_TIMEOUT ms\n"
  "\n"
  "uint16 msg_sequence";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__UlogStreamAck__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__UlogStreamAck__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 359, 359},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__UlogStreamAck__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__UlogStreamAck__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
