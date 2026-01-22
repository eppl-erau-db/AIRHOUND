// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LoggerStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/logger_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LoggerStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xab, 0x86, 0xe3, 0x3d, 0x7e, 0x94, 0xbb, 0x26,
      0xea, 0x8a, 0x14, 0x6f, 0x2a, 0xc5, 0x55, 0x53,
      0xf7, 0xe6, 0xdd, 0x42, 0xe2, 0x8e, 0xf8, 0xe2,
      0xe9, 0xc7, 0x00, 0x98, 0x0d, 0x4b, 0x07, 0x9a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LoggerStatus__TYPE_NAME[] = "px4_msgs/msg/LoggerStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__type[] = "type";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__backend[] = "backend";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__is_logging[] = "is_logging";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__total_written_kb[] = "total_written_kb";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__write_rate_kb_s[] = "write_rate_kb_s";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__dropouts[] = "dropouts";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__message_gaps[] = "message_gaps";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__buffer_used_bytes[] = "buffer_used_bytes";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__buffer_size_bytes[] = "buffer_size_bytes";
static char px4_msgs__msg__LoggerStatus__FIELD_NAME__num_messages[] = "num_messages";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LoggerStatus__FIELDS[] = {
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__backend, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__is_logging, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__total_written_kb, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__write_rate_kb_s, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__dropouts, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__message_gaps, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__buffer_used_bytes, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__buffer_size_bytes, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LoggerStatus__FIELD_NAME__num_messages, 12, 12},
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
px4_msgs__msg__LoggerStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LoggerStatus__TYPE_NAME, 25, 25},
      {px4_msgs__msg__LoggerStatus__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp               # time since system start (microseconds)\n"
  "\n"
  "uint8 LOGGER_TYPE_FULL    = 0  # Normal, full size log\n"
  "uint8 LOGGER_TYPE_MISSION = 1  # reduced mission log (e.g. for geotagging)\n"
  "uint8 type\n"
  "\n"
  "uint8 BACKEND_FILE    = 1\n"
  "uint8 BACKEND_MAVLINK = 2\n"
  "uint8 BACKEND_ALL     = 3\n"
  "uint8 backend\n"
  "\n"
  "bool is_logging\n"
  "\n"
  "float32 total_written_kb       # total written to log in kiloBytes\n"
  "float32 write_rate_kb_s        # write rate in kiloBytes/s\n"
  "\n"
  "uint32 dropouts                # number of failed buffer writes due to buffer overflow\n"
  "uint32 message_gaps            # messages misssed\n"
  "\n"
  "uint32 buffer_used_bytes       # current buffer fill in Bytes\n"
  "uint32 buffer_size_bytes       # total buffer size in Bytes\n"
  "\n"
  "uint8 num_messages";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LoggerStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LoggerStatus__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 732, 732},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LoggerStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LoggerStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
