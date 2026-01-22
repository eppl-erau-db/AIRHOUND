// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ParameterSetValueResponse.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/parameter_set_value_response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ParameterSetValueResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0xc2, 0xf1, 0xee, 0xb2, 0x7b, 0x42, 0x3a,
      0xe7, 0xc7, 0xa3, 0x1a, 0x8b, 0xf5, 0x79, 0x73,
      0x77, 0xa8, 0x9d, 0x89, 0x08, 0x6e, 0xb7, 0x33,
      0xb0, 0xe7, 0x16, 0xde, 0x90, 0xe2, 0x1f, 0x8f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ParameterSetValueResponse__TYPE_NAME[] = "px4_msgs/msg/ParameterSetValueResponse";

// Define type names, field names, and default values
static char px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__request_timestamp[] = "request_timestamp";
static char px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__parameter_index[] = "parameter_index";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ParameterSetValueResponse__FIELDS[] = {
  {
    {px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__request_timestamp, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterSetValueResponse__FIELD_NAME__parameter_index, 15, 15},
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
px4_msgs__msg__ParameterSetValueResponse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ParameterSetValueResponse__TYPE_NAME, 38, 38},
      {px4_msgs__msg__ParameterSetValueResponse__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ParameterSetValueResponse : Response to a set value request by either primary or secondary\n"
  "\n"
  "uint64 timestamp\n"
  "uint64 request_timestamp\n"
  "uint16 parameter_index\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4\n"
  "\n"
  "# TOPICS parameter_set_value_response parameter_remote_set_value_response parameter_primary_set_value_response";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ParameterSetValueResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ParameterSetValueResponse__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 299, 299},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ParameterSetValueResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ParameterSetValueResponse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
