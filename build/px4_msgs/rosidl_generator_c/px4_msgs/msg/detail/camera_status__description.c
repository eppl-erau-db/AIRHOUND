// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CameraStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/camera_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CameraStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x69, 0xc4, 0x75, 0x78, 0x2f, 0x32, 0x2f, 0xc7,
      0xa7, 0x5d, 0xd3, 0x68, 0x8a, 0x73, 0xe0, 0xf5,
      0xf7, 0x99, 0xf9, 0x0b, 0x98, 0xe1, 0xa1, 0x8e,
      0x89, 0x5c, 0x62, 0x51, 0x82, 0x0a, 0xa5, 0x51,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CameraStatus__TYPE_NAME[] = "px4_msgs/msg/CameraStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__CameraStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CameraStatus__FIELD_NAME__active_sys_id[] = "active_sys_id";
static char px4_msgs__msg__CameraStatus__FIELD_NAME__active_comp_id[] = "active_comp_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CameraStatus__FIELDS[] = {
  {
    {px4_msgs__msg__CameraStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CameraStatus__FIELD_NAME__active_sys_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CameraStatus__FIELD_NAME__active_comp_id, 14, 14},
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
px4_msgs__msg__CameraStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CameraStatus__TYPE_NAME, 25, 25},
      {px4_msgs__msg__CameraStatus__FIELDS, 3, 3},
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
  "uint8 active_sys_id\\t\\t# mavlink system id of the currently active camera\n"
  "uint8 active_comp_id \\t# mavlink component id of currently active camera";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CameraStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CameraStatus__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 204, 204},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CameraStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CameraStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
