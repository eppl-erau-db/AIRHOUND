// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverAttitudeSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_attitude_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverAttitudeSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1c, 0x66, 0x7e, 0xb4, 0x12, 0x4a, 0x93, 0x24,
      0xec, 0x10, 0xde, 0x51, 0xf4, 0xfb, 0x81, 0x88,
      0xd6, 0x1c, 0x56, 0x46, 0xac, 0x41, 0x9f, 0xe9,
      0xde, 0x55, 0x40, 0xec, 0x9b, 0xd3, 0x19, 0x34,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverAttitudeSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverAttitudeSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverAttitudeSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverAttitudeSetpoint__FIELD_NAME__yaw_setpoint[] = "yaw_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverAttitudeSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverAttitudeSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverAttitudeSetpoint__FIELD_NAME__yaw_setpoint, 12, 12},
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
px4_msgs__msg__RoverAttitudeSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverAttitudeSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__RoverAttitudeSetpoint__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Attitude Setpoint\n"
  "\n"
  "uint64 timestamp      # [us] Time since system start\n"
  "float32 yaw_setpoint  # [rad] [@range -inf, inf] [@frame NED] Yaw setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverAttitudeSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverAttitudeSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 155, 155},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverAttitudeSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverAttitudeSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
