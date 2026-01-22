// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Ping.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/ping__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Ping__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0xed, 0x5b, 0x2a, 0xc8, 0xa4, 0x81, 0x4b,
      0x99, 0xba, 0xa7, 0xb5, 0x54, 0xca, 0x12, 0x7f,
      0x2c, 0x15, 0x8b, 0x92, 0x92, 0xd5, 0x79, 0x84,
      0x2c, 0xc7, 0x72, 0x4d, 0xd7, 0x3a, 0xba, 0x6a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Ping__TYPE_NAME[] = "px4_msgs/msg/Ping";

// Define type names, field names, and default values
static char px4_msgs__msg__Ping__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Ping__FIELD_NAME__ping_time[] = "ping_time";
static char px4_msgs__msg__Ping__FIELD_NAME__ping_sequence[] = "ping_sequence";
static char px4_msgs__msg__Ping__FIELD_NAME__dropped_packets[] = "dropped_packets";
static char px4_msgs__msg__Ping__FIELD_NAME__rtt_ms[] = "rtt_ms";
static char px4_msgs__msg__Ping__FIELD_NAME__system_id[] = "system_id";
static char px4_msgs__msg__Ping__FIELD_NAME__component_id[] = "component_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Ping__FIELDS[] = {
  {
    {px4_msgs__msg__Ping__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__ping_time, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__ping_sequence, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__dropped_packets, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__rtt_ms, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__system_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ping__FIELD_NAME__component_id, 12, 12},
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
px4_msgs__msg__Ping__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Ping__TYPE_NAME, 17, 17},
      {px4_msgs__msg__Ping__FIELDS, 7, 7},
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
  "uint64 ping_time\\t\\t\\t# Timestamp of the ping packet\n"
  "uint32 ping_sequence\\t\\t# Sequence number of the ping packet\n"
  "uint32 dropped_packets\\t\\t# Number of dropped ping packets\n"
  "float32 rtt_ms\\t\\t\\t\\t# Round trip time (in ms)\n"
  "uint8 system_id\\t\\t\\t\\t# System ID of the remote system\n"
  "uint8 component_id\\t\\t\\t# Component ID of the remote system";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Ping__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Ping__TYPE_NAME, 17, 17},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 379, 379},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Ping__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Ping__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
