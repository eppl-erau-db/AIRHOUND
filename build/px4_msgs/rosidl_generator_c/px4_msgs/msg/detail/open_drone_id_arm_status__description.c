// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OpenDroneIdArmStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/open_drone_id_arm_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OpenDroneIdArmStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8c, 0x71, 0x80, 0x0b, 0x11, 0xae, 0xfc, 0xc8,
      0x5e, 0xaf, 0x03, 0xa0, 0x9a, 0x16, 0x2e, 0x8c,
      0x58, 0x1c, 0x33, 0x66, 0x1e, 0x86, 0x54, 0xc6,
      0x4d, 0xf2, 0x5e, 0x9f, 0x83, 0xb4, 0xf1, 0x58,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OpenDroneIdArmStatus__TYPE_NAME[] = "px4_msgs/msg/OpenDroneIdArmStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__status[] = "status";
static char px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OpenDroneIdArmStatus__FIELDS[] = {
  {
    {px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OpenDroneIdArmStatus__FIELD_NAME__error, 5, 5},
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
px4_msgs__msg__OpenDroneIdArmStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OpenDroneIdArmStatus__TYPE_NAME, 33, 33},
      {px4_msgs__msg__OpenDroneIdArmStatus__FIELDS, 3, 3},
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
  "uint8 status\n"
  "char[50] error";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OpenDroneIdArmStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OpenDroneIdArmStatus__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OpenDroneIdArmStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OpenDroneIdArmStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
