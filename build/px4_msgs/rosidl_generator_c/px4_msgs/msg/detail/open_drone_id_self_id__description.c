// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OpenDroneIdSelfId.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/open_drone_id_self_id__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OpenDroneIdSelfId__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3c, 0xd5, 0x6e, 0x54, 0x94, 0x1d, 0x04, 0x49,
      0x4b, 0x5a, 0xdf, 0x60, 0x47, 0x5a, 0xd6, 0xd3,
      0x28, 0xb5, 0xbf, 0x2f, 0x2f, 0x0f, 0xba, 0x11,
      0x14, 0x23, 0x5a, 0xa2, 0x2d, 0xc9, 0x3d, 0xe1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OpenDroneIdSelfId__TYPE_NAME[] = "px4_msgs/msg/OpenDroneIdSelfId";

// Define type names, field names, and default values
static char px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__id_or_mac[] = "id_or_mac";
static char px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__description_type[] = "description_type";
static char px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__description[] = "description";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OpenDroneIdSelfId__FIELDS[] = {
  {
    {px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__id_or_mac, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      20,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__description_type, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdSelfId__FIELD_NAME__description, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      23,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OpenDroneIdSelfId__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OpenDroneIdSelfId__TYPE_NAME, 30, 30},
      {px4_msgs__msg__OpenDroneIdSelfId__FIELDS, 4, 4},
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
  "uint8 description_type\n"
  "char[23] description";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OpenDroneIdSelfId__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OpenDroneIdSelfId__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 81, 81},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OpenDroneIdSelfId__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OpenDroneIdSelfId__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
