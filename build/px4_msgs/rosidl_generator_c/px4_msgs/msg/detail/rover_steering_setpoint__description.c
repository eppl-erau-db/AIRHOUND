// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverSteeringSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_steering_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverSteeringSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x85, 0x3d, 0x7c, 0x62, 0xb4, 0xdd, 0x52, 0xf1,
      0x10, 0xb8, 0x8b, 0x7a, 0x50, 0xbb, 0xbe, 0xd0,
      0xa4, 0x70, 0xa8, 0xa0, 0x63, 0xeb, 0x73, 0x8b,
      0x7c, 0x99, 0xe5, 0x3c, 0xeb, 0xce, 0x1b, 0x45,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverSteeringSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverSteeringSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverSteeringSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverSteeringSetpoint__FIELD_NAME__normalized_steering_setpoint[] = "normalized_steering_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverSteeringSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverSteeringSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSteeringSetpoint__FIELD_NAME__normalized_steering_setpoint, 28, 28},
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
px4_msgs__msg__RoverSteeringSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverSteeringSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__RoverSteeringSetpoint__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Steering setpoint\n"
  "\n"
  "uint64 timestamp                      # [us] Time since system start\n"
  "float32 normalized_steering_setpoint  # [-] [@range -1 (Left), 1 (Right)] [@frame Body] Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverSteeringSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverSteeringSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 306, 306},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverSteeringSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverSteeringSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
