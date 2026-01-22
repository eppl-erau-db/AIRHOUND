// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ParameterResetRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/parameter_reset_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ParameterResetRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x45, 0xc0, 0x56, 0xe3, 0x06, 0xe4, 0xa0, 0x9d,
      0x21, 0x02, 0xf5, 0x76, 0xb4, 0x11, 0x2a, 0x2d,
      0x70, 0x5b, 0xd4, 0xdc, 0xaf, 0x01, 0x3a, 0x1f,
      0x01, 0x40, 0x21, 0xc5, 0x5e, 0xdc, 0xed, 0xcb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ParameterResetRequest__TYPE_NAME[] = "px4_msgs/msg/ParameterResetRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__ParameterResetRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ParameterResetRequest__FIELD_NAME__parameter_index[] = "parameter_index";
static char px4_msgs__msg__ParameterResetRequest__FIELD_NAME__reset_all[] = "reset_all";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ParameterResetRequest__FIELDS[] = {
  {
    {px4_msgs__msg__ParameterResetRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterResetRequest__FIELD_NAME__parameter_index, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ParameterResetRequest__FIELD_NAME__reset_all, 9, 9},
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
px4_msgs__msg__ParameterResetRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ParameterResetRequest__TYPE_NAME, 34, 34},
      {px4_msgs__msg__ParameterResetRequest__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ParameterResetRequest : Used by the primary to reset one or all parameter value(s) on the remote\n"
  "\n"
  "uint64 timestamp\n"
  "uint16 parameter_index\n"
  "\n"
  "bool reset_all              # If this is true then ignore parameter_index\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ParameterResetRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ParameterResetRequest__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 243, 243},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ParameterResetRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ParameterResetRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
