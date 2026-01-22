// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TimesyncStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/timesync_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TimesyncStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0x1c, 0xb0, 0x5a, 0x03, 0x38, 0x18, 0xad,
      0x80, 0x25, 0xbc, 0xc7, 0xb2, 0xa0, 0x6a, 0xb5,
      0x36, 0xcb, 0x2f, 0x93, 0xf9, 0xd1, 0xf3, 0x13,
      0x62, 0xc2, 0xf9, 0xbc, 0xc6, 0x41, 0xf1, 0x61,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TimesyncStatus__TYPE_NAME[] = "px4_msgs/msg/TimesyncStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__source_protocol[] = "source_protocol";
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__remote_timestamp[] = "remote_timestamp";
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__observed_offset[] = "observed_offset";
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__estimated_offset[] = "estimated_offset";
static char px4_msgs__msg__TimesyncStatus__FIELD_NAME__round_trip_time[] = "round_trip_time";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TimesyncStatus__FIELDS[] = {
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__source_protocol, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__remote_timestamp, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__observed_offset, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__estimated_offset, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TimesyncStatus__FIELD_NAME__round_trip_time, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TimesyncStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TimesyncStatus__TYPE_NAME, 27, 27},
      {px4_msgs__msg__TimesyncStatus__FIELDS, 6, 6},
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
  "\n"
  "uint8 SOURCE_PROTOCOL_UNKNOWN = 0\n"
  "uint8 SOURCE_PROTOCOL_MAVLINK = 1\n"
  "uint8 SOURCE_PROTOCOL_DDS     = 2\n"
  "uint8 source_protocol\\t\\t\\t# timesync source\n"
  "\n"
  "uint64 remote_timestamp\\t\\t\\t# remote system timestamp (microseconds)\n"
  "int64 observed_offset\\t\\t\\t# raw time offset directly observed from this timesync packet (microseconds)\n"
  "int64 estimated_offset\\t\\t\\t# smoothed time offset between companion system and PX4 (microseconds)\n"
  "uint32 round_trip_time\\t\\t\\t# round trip time of this timesync packet (microseconds)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TimesyncStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TimesyncStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 552, 552},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TimesyncStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TimesyncStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
