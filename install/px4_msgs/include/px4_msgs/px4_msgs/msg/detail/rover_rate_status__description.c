// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverRateStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_rate_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverRateStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7e, 0xf8, 0x86, 0x1b, 0x7a, 0x02, 0xf7, 0x14,
      0x1e, 0xd2, 0xee, 0x9f, 0xca, 0xbe, 0x5f, 0xbe,
      0x81, 0xa6, 0x40, 0xa2, 0x82, 0xe3, 0x71, 0x95,
      0x9d, 0x90, 0x06, 0xa8, 0x80, 0x5c, 0xf4, 0xf4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverRateStatus__TYPE_NAME[] = "px4_msgs/msg/RoverRateStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverRateStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverRateStatus__FIELD_NAME__measured_yaw_rate[] = "measured_yaw_rate";
static char px4_msgs__msg__RoverRateStatus__FIELD_NAME__adjusted_yaw_rate_setpoint[] = "adjusted_yaw_rate_setpoint";
static char px4_msgs__msg__RoverRateStatus__FIELD_NAME__pid_yaw_rate_integral[] = "pid_yaw_rate_integral";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverRateStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RoverRateStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverRateStatus__FIELD_NAME__measured_yaw_rate, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverRateStatus__FIELD_NAME__adjusted_yaw_rate_setpoint, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverRateStatus__FIELD_NAME__pid_yaw_rate_integral, 21, 21},
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
px4_msgs__msg__RoverRateStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverRateStatus__TYPE_NAME, 28, 28},
      {px4_msgs__msg__RoverRateStatus__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Rate Status\n"
  "\n"
  "uint64 timestamp                    # [us] Time since system start\n"
  "float32 measured_yaw_rate           # [rad/s] [@range -inf, inf] [@frame NED] Measured yaw rate\n"
  "float32 adjusted_yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint that is being tracked (Applied slew rates)\n"
  "float32 pid_yaw_rate_integral       # [-] [@range -1, 1] Integral of the PID for the closed loop yaw rate controller";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverRateStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverRateStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 440, 440},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverRateStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverRateStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
