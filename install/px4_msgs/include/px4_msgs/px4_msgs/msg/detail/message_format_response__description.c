// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MessageFormatResponse.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/message_format_response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MessageFormatResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd9, 0xaa, 0x83, 0x91, 0x8b, 0xe5, 0x68, 0x0d,
      0xc5, 0x43, 0x71, 0x32, 0x09, 0xab, 0x43, 0xd0,
      0x20, 0x11, 0xe7, 0xc9, 0x62, 0xe2, 0x5f, 0xad,
      0x23, 0x2e, 0x75, 0x23, 0x03, 0x3d, 0xd6, 0x62,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MessageFormatResponse__TYPE_NAME[] = "px4_msgs/msg/MessageFormatResponse";

// Define type names, field names, and default values
static char px4_msgs__msg__MessageFormatResponse__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MessageFormatResponse__FIELD_NAME__protocol_version[] = "protocol_version";
static char px4_msgs__msg__MessageFormatResponse__FIELD_NAME__topic_name[] = "topic_name";
static char px4_msgs__msg__MessageFormatResponse__FIELD_NAME__success[] = "success";
static char px4_msgs__msg__MessageFormatResponse__FIELD_NAME__message_hash[] = "message_hash";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MessageFormatResponse__FIELDS[] = {
  {
    {px4_msgs__msg__MessageFormatResponse__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatResponse__FIELD_NAME__protocol_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatResponse__FIELD_NAME__topic_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      50,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatResponse__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatResponse__FIELD_NAME__message_hash, 12, 12},
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
px4_msgs__msg__MessageFormatResponse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MessageFormatResponse__TYPE_NAME, 34, 34},
      {px4_msgs__msg__MessageFormatResponse__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "# Response from PX4 with the format of a message\n"
  "\n"
  "uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp\n"
  "\n"
  "char[50] topic_name  # E.g. /fmu/in/vehicle_command\n"
  "\n"
  "bool success\n"
  "uint32 message_hash # hash over all message fields";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MessageFormatResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MessageFormatResponse__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 376, 376},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MessageFormatResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MessageFormatResponse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
