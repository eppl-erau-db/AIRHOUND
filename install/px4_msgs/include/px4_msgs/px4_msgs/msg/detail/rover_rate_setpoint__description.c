// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverRateSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_rate_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverRateSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0xa5, 0x25, 0x6b, 0x31, 0x5a, 0xfb, 0xf6,
      0xd2, 0x93, 0xb8, 0x94, 0xbd, 0xc6, 0x7c, 0x84,
      0x72, 0x4e, 0x45, 0x27, 0x14, 0x0d, 0x39, 0x34,
      0x8c, 0x60, 0x16, 0xbf, 0xef, 0xa3, 0x6a, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverRateSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverRateSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverRateSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverRateSetpoint__FIELD_NAME__yaw_rate_setpoint[] = "yaw_rate_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverRateSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverRateSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverRateSetpoint__FIELD_NAME__yaw_rate_setpoint, 17, 17},
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
px4_msgs__msg__RoverRateSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverRateSetpoint__TYPE_NAME, 30, 30},
      {px4_msgs__msg__RoverRateSetpoint__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Rate setpoint\n"
  "\n"
  "uint64 timestamp           # [us] Time since system start\n"
  "float32 yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverRateSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverRateSetpoint__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 168, 168},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverRateSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverRateSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
