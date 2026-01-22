// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/UlogStream.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/ulog_stream__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__UlogStream__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0c, 0x56, 0x9a, 0x27, 0x1d, 0xa7, 0x17, 0xd6,
      0x23, 0x63, 0x74, 0xe7, 0xec, 0xe4, 0x7a, 0xb4,
      0x18, 0x4c, 0x23, 0x29, 0x49, 0x78, 0x8b, 0xa6,
      0x2f, 0x7f, 0xa6, 0x34, 0x6d, 0xa7, 0x16, 0xd9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__UlogStream__TYPE_NAME[] = "px4_msgs/msg/UlogStream";

// Define type names, field names, and default values
static char px4_msgs__msg__UlogStream__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__UlogStream__FIELD_NAME__length[] = "length";
static char px4_msgs__msg__UlogStream__FIELD_NAME__first_message_offset[] = "first_message_offset";
static char px4_msgs__msg__UlogStream__FIELD_NAME__msg_sequence[] = "msg_sequence";
static char px4_msgs__msg__UlogStream__FIELD_NAME__flags[] = "flags";
static char px4_msgs__msg__UlogStream__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__UlogStream__FIELDS[] = {
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__length, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__first_message_offset, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__msg_sequence, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__flags, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UlogStream__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      249,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__UlogStream__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__UlogStream__TYPE_NAME, 23, 23},
      {px4_msgs__msg__UlogStream__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Message to stream ULog data from the logger. Corresponds to the LOGGING_DATA\n"
  "# mavlink message\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# flags bitmasks\n"
  "uint8 FLAGS_NEED_ACK = 1\\t# if set, this message requires to be acked.\n"
  "\\t\\t\\t\\t# Acked messages are published synchronous: a\n"
  "\\t\\t\\t\\t# publisher waits for an ack before sending the\n"
  "\\t\\t\\t\\t# next message\n"
  "\n"
  "uint8 length\\t\\t\\t# length of data\n"
  "uint8 first_message_offset\\t# offset into data where first message starts. This\n"
  "\\t\\t\\t\\t# can be used for recovery, when a previous message got lost\n"
  "uint16 msg_sequence\\t\\t# allows determine drops\n"
  "uint8 flags\\t\\t\\t# see FLAGS_*\n"
  "uint8[249] data\\t\\t# ulog data\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 16\\t# TODO: we might be able to reduce this if mavlink polled on the topic";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__UlogStream__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__UlogStream__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 746, 746},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__UlogStream__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__UlogStream__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
