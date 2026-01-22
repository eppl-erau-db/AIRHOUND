// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OpenDroneIdOperatorId.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/open_drone_id_operator_id__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OpenDroneIdOperatorId__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x8e, 0x01, 0x46, 0x6d, 0x7e, 0x49, 0x8d,
      0xb0, 0xa4, 0x4f, 0xb8, 0xe9, 0xfb, 0xc7, 0xb6,
      0x65, 0x57, 0x8e, 0xdc, 0x77, 0x83, 0xf8, 0x96,
      0x90, 0xf2, 0x9c, 0x8d, 0x39, 0x1d, 0x9c, 0x9a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OpenDroneIdOperatorId__TYPE_NAME[] = "px4_msgs/msg/OpenDroneIdOperatorId";

// Define type names, field names, and default values
static char px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__id_or_mac[] = "id_or_mac";
static char px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__operator_id_type[] = "operator_id_type";
static char px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__operator_id[] = "operator_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OpenDroneIdOperatorId__FIELDS[] = {
  {
    {px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__id_or_mac, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      20,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__operator_id_type, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdOperatorId__FIELD_NAME__operator_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      20,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OpenDroneIdOperatorId__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OpenDroneIdOperatorId__TYPE_NAME, 34, 34},
      {px4_msgs__msg__OpenDroneIdOperatorId__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\n"
  "uint8[20] id_or_mac\n"
  "uint8 operator_id_type\n"
  "char[20] operator_id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OpenDroneIdOperatorId__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OpenDroneIdOperatorId__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 81, 81},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OpenDroneIdOperatorId__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OpenDroneIdOperatorId__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
