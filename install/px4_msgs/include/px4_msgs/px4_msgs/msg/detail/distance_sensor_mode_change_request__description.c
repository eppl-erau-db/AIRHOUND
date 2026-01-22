// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DistanceSensorModeChangeRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/distance_sensor_mode_change_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DistanceSensorModeChangeRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x32, 0x89, 0x1c, 0x76, 0xb1, 0x4a, 0xc4, 0xc2,
      0x08, 0xc4, 0x22, 0x29, 0x53, 0x82, 0x0c, 0x93,
      0xdc, 0x79, 0x68, 0x5c, 0xfc, 0x09, 0xe6, 0xbf,
      0x04, 0x5a, 0xae, 0xdc, 0x4e, 0x83, 0x4d, 0xc8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DistanceSensorModeChangeRequest__TYPE_NAME[] = "px4_msgs/msg/DistanceSensorModeChangeRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__DistanceSensorModeChangeRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DistanceSensorModeChangeRequest__FIELD_NAME__request_on_off[] = "request_on_off";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DistanceSensorModeChangeRequest__FIELDS[] = {
  {
    {px4_msgs__msg__DistanceSensorModeChangeRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DistanceSensorModeChangeRequest__FIELD_NAME__request_on_off, 14, 14},
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
px4_msgs__msg__DistanceSensorModeChangeRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DistanceSensorModeChangeRequest__TYPE_NAME, 44, 44},
      {px4_msgs__msg__DistanceSensorModeChangeRequest__FIELDS, 2, 2},
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
  "uint8 request_on_off \\t\\t\\t# request to disable/enable the distance sensor\n"
  "uint8 REQUEST_OFF = 0\n"
  "uint8 REQUEST_ON  = 1";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DistanceSensorModeChangeRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DistanceSensorModeChangeRequest__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 176, 176},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DistanceSensorModeChangeRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DistanceSensorModeChangeRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
