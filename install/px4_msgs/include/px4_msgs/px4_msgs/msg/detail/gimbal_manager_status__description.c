// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GimbalManagerStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gimbal_manager_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GimbalManagerStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0xa6, 0x99, 0x84, 0xed, 0x1c, 0x99, 0x0b,
      0x1a, 0x59, 0x18, 0xd0, 0xab, 0xc1, 0x45, 0x6e,
      0x8d, 0xc9, 0x2f, 0x0f, 0x33, 0x03, 0x1c, 0xee,
      0x17, 0xcc, 0xde, 0x4b, 0x94, 0xa1, 0x39, 0x32,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GimbalManagerStatus__TYPE_NAME[] = "px4_msgs/msg/GimbalManagerStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__flags[] = "flags";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__gimbal_device_id[] = "gimbal_device_id";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__primary_control_sysid[] = "primary_control_sysid";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__primary_control_compid[] = "primary_control_compid";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__secondary_control_sysid[] = "secondary_control_sysid";
static char px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__secondary_control_compid[] = "secondary_control_compid";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GimbalManagerStatus__FIELDS[] = {
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__flags, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__gimbal_device_id, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__primary_control_sysid, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__primary_control_compid, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__secondary_control_sysid, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerStatus__FIELD_NAME__secondary_control_compid, 24, 24},
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
px4_msgs__msg__GimbalManagerStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GimbalManagerStatus__TYPE_NAME, 32, 32},
      {px4_msgs__msg__GimbalManagerStatus__FIELDS, 7, 7},
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
  "\n"
  "uint32 flags\n"
  "uint8 gimbal_device_id\n"
  "uint8 primary_control_sysid\n"
  "uint8 primary_control_compid\n"
  "uint8 secondary_control_sysid\n"
  "uint8 secondary_control_compid";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GimbalManagerStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GimbalManagerStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 214, 214},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GimbalManagerStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GimbalManagerStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
