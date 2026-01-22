// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverAttitudeStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_attitude_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverAttitudeStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3e, 0xc9, 0xdd, 0x8c, 0x59, 0xe9, 0xc8, 0x91,
      0xe5, 0xe0, 0x56, 0x0a, 0x77, 0x31, 0xd1, 0x6d,
      0x62, 0xef, 0xce, 0xb4, 0x93, 0x0a, 0xe7, 0xf5,
      0x0e, 0xfe, 0xc2, 0x21, 0x8c, 0x6a, 0x6f, 0xb9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverAttitudeStatus__TYPE_NAME[] = "px4_msgs/msg/RoverAttitudeStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__measured_yaw[] = "measured_yaw";
static char px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__adjusted_yaw_setpoint[] = "adjusted_yaw_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverAttitudeStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__measured_yaw, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverAttitudeStatus__FIELD_NAME__adjusted_yaw_setpoint, 21, 21},
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
px4_msgs__msg__RoverAttitudeStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverAttitudeStatus__TYPE_NAME, 32, 32},
      {px4_msgs__msg__RoverAttitudeStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Attitude Status\n"
  "\n"
  "uint64 timestamp               # [us] Time since system start\n"
  "float32 measured_yaw           # [rad] [@range -pi, pi] [@frame NED]Measured yaw\n"
  "float32 adjusted_yaw_setpoint  # [rad] [@range -pi, pi] [@frame NED] Yaw setpoint that is being tracked (Applied slew rates)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverAttitudeStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverAttitudeStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 293, 293},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverAttitudeStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverAttitudeStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
