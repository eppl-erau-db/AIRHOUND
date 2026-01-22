// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverThrottleSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_throttle_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverThrottleSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x42, 0x31, 0x97, 0x1e, 0x31, 0xd3, 0x78, 0x8c,
      0xe8, 0x65, 0x70, 0x3f, 0xc6, 0x29, 0xe4, 0x55,
      0x3f, 0x31, 0x29, 0xa7, 0x0f, 0xc7, 0x24, 0xa5,
      0x28, 0x70, 0x16, 0x27, 0xb3, 0xc2, 0x52, 0x5c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverThrottleSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverThrottleSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__throttle_body_x[] = "throttle_body_x";
static char px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__throttle_body_y[] = "throttle_body_y";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverThrottleSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__throttle_body_x, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverThrottleSetpoint__FIELD_NAME__throttle_body_y, 15, 15},
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
px4_msgs__msg__RoverThrottleSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverThrottleSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__RoverThrottleSetpoint__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Throttle setpoint\n"
  "\n"
  "uint64 timestamp         # [us] Time since system start\n"
  "float32 throttle_body_x  # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis\n"
  "float32 throttle_body_y  # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverThrottleSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverThrottleSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 357, 357},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverThrottleSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverThrottleSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
