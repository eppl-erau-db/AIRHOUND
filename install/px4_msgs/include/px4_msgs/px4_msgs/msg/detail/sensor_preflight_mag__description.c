// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorPreflightMag.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_preflight_mag__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorPreflightMag__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc0, 0x03, 0x02, 0x8f, 0xab, 0x05, 0x9a, 0x33,
      0x76, 0x15, 0xbf, 0x81, 0x66, 0x8f, 0x40, 0xdf,
      0x1e, 0x41, 0x95, 0x1a, 0x55, 0xe2, 0x45, 0xaa,
      0xb1, 0x55, 0xbc, 0x81, 0x86, 0x01, 0xf5, 0xe2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorPreflightMag__TYPE_NAME[] = "px4_msgs/msg/SensorPreflightMag";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorPreflightMag__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorPreflightMag__FIELD_NAME__mag_inconsistency_angle[] = "mag_inconsistency_angle";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorPreflightMag__FIELDS[] = {
  {
    {px4_msgs__msg__SensorPreflightMag__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorPreflightMag__FIELD_NAME__mag_inconsistency_angle, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorPreflightMag__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorPreflightMag__TYPE_NAME, 31, 31},
      {px4_msgs__msg__SensorPreflightMag__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Pre-flight sensor check metrics.\n"
  "# The topic will not be updated when the vehicle is armed\n"
  "#\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 mag_inconsistency_angle # maximum angle between magnetometer instance field vectors in radians.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorPreflightMag__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorPreflightMag__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 260, 260},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorPreflightMag__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorPreflightMag__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
