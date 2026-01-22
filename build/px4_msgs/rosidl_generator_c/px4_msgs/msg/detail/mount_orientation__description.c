// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MountOrientation.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/mount_orientation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MountOrientation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4f, 0xd8, 0x46, 0x11, 0x8d, 0x5e, 0xbb, 0xe4,
      0x2b, 0x18, 0xa0, 0x50, 0xc4, 0x64, 0x89, 0xb2,
      0xbb, 0x78, 0xea, 0xe5, 0x24, 0xa7, 0xe6, 0xf9,
      0x3a, 0x10, 0x07, 0x74, 0xa0, 0xd5, 0xeb, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MountOrientation__TYPE_NAME[] = "px4_msgs/msg/MountOrientation";

// Define type names, field names, and default values
static char px4_msgs__msg__MountOrientation__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MountOrientation__FIELD_NAME__attitude_euler_angle[] = "attitude_euler_angle";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MountOrientation__FIELDS[] = {
  {
    {px4_msgs__msg__MountOrientation__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MountOrientation__FIELD_NAME__attitude_euler_angle, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__MountOrientation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MountOrientation__TYPE_NAME, 29, 29},
      {px4_msgs__msg__MountOrientation__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t# time since system start (microseconds)\n"
  "float32[3] attitude_euler_angle\\t\\t# Attitude/direction of the mount as euler angles in rad";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MountOrientation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MountOrientation__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MountOrientation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MountOrientation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
