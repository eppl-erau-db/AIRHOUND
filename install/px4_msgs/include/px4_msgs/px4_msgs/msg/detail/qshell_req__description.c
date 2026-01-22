// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/QshellReq.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/qshell_req__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__QshellReq__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0xea, 0x27, 0x9f, 0xbf, 0xfd, 0xb4, 0x2d,
      0xa4, 0x69, 0x70, 0x34, 0xc9, 0xae, 0x7d, 0x92,
      0xa4, 0x78, 0xa8, 0x1c, 0x45, 0x2e, 0xd5, 0xb0,
      0x8b, 0xe4, 0x71, 0xb3, 0x32, 0x4a, 0xcb, 0x86,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__QshellReq__TYPE_NAME[] = "px4_msgs/msg/QshellReq";

// Define type names, field names, and default values
static char px4_msgs__msg__QshellReq__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__QshellReq__FIELD_NAME__cmd[] = "cmd";
static char px4_msgs__msg__QshellReq__FIELD_NAME__strlen[] = "strlen";
static char px4_msgs__msg__QshellReq__FIELD_NAME__request_sequence[] = "request_sequence";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__QshellReq__FIELDS[] = {
  {
    {px4_msgs__msg__QshellReq__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__QshellReq__FIELD_NAME__cmd, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      100,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__QshellReq__FIELD_NAME__strlen, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__QshellReq__FIELD_NAME__request_sequence, 16, 16},
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
px4_msgs__msg__QshellReq__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__QshellReq__TYPE_NAME, 22, 22},
      {px4_msgs__msg__QshellReq__FIELDS, 4, 4},
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
  "char[100] cmd\n"
  "uint32 MAX_STRLEN = 100\n"
  "uint32 strlen\n"
  "uint32 request_sequence";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__QshellReq__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__QshellReq__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 135, 135},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__QshellReq__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__QshellReq__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
