// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MessageFormatRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/message_format_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MessageFormatRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0x0f, 0x0e, 0xa5, 0x15, 0xbc, 0x83, 0x07,
      0x66, 0x06, 0xad, 0xbd, 0x5d, 0x6c, 0xac, 0x79,
      0x8f, 0xcb, 0xbd, 0x9d, 0x64, 0x9c, 0x96, 0xc4,
      0x1f, 0x00, 0x0b, 0xc5, 0x14, 0x3a, 0x9c, 0x9d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MessageFormatRequest__TYPE_NAME[] = "px4_msgs/msg/MessageFormatRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__MessageFormatRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MessageFormatRequest__FIELD_NAME__protocol_version[] = "protocol_version";
static char px4_msgs__msg__MessageFormatRequest__FIELD_NAME__topic_name[] = "topic_name";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MessageFormatRequest__FIELDS[] = {
  {
    {px4_msgs__msg__MessageFormatRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatRequest__FIELD_NAME__protocol_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MessageFormatRequest__FIELD_NAME__topic_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      50,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__MessageFormatRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MessageFormatRequest__TYPE_NAME, 33, 33},
      {px4_msgs__msg__MessageFormatRequest__FIELDS, 3, 3},
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
  "# Request to PX4 to get the hash of a message, to check for message compatibility\n"
  "\n"
  "uint16 LATEST_PROTOCOL_VERSION = 1 # Current version of this protocol. Increase this whenever the MessageFormatRequest or MessageFormatResponse changes.\n"
  "\n"
  "uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp\n"
  "\n"
  "char[50] topic_name  # E.g. /fmu/in/vehicle_command";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MessageFormatRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MessageFormatRequest__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 498, 498},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MessageFormatRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MessageFormatRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
