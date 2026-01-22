// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CameraTrigger.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/camera_trigger__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CameraTrigger__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x17, 0xea, 0xe4, 0x0c, 0x6d, 0xaa, 0x19, 0x30,
      0xb0, 0xe0, 0xac, 0xf8, 0x8d, 0x22, 0xfa, 0x44,
      0x78, 0x4f, 0xa7, 0x6c, 0x3d, 0x61, 0xd9, 0xe9,
      0x7c, 0x0e, 0x43, 0xea, 0xd1, 0x4f, 0x51, 0x08,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CameraTrigger__TYPE_NAME[] = "px4_msgs/msg/CameraTrigger";

// Define type names, field names, and default values
static char px4_msgs__msg__CameraTrigger__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CameraTrigger__FIELD_NAME__timestamp_utc[] = "timestamp_utc";
static char px4_msgs__msg__CameraTrigger__FIELD_NAME__seq[] = "seq";
static char px4_msgs__msg__CameraTrigger__FIELD_NAME__feedback[] = "feedback";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CameraTrigger__FIELDS[] = {
  {
    {px4_msgs__msg__CameraTrigger__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CameraTrigger__FIELD_NAME__timestamp_utc, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CameraTrigger__FIELD_NAME__seq, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CameraTrigger__FIELD_NAME__feedback, 8, 8},
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
px4_msgs__msg__CameraTrigger__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CameraTrigger__TYPE_NAME, 26, 26},
      {px4_msgs__msg__CameraTrigger__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "uint64 timestamp_utc # UTC timestamp\n"
  "\n"
  "uint32 seq\\t\\t# Image sequence number\n"
  "bool feedback\\t# Trigger feedback from camera\n"
  "\n"
  "uint32 ORB_QUEUE_LENGTH = 2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CameraTrigger__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CameraTrigger__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 206, 206},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CameraTrigger__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CameraTrigger__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
