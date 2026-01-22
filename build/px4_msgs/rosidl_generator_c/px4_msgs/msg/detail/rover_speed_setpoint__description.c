// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverSpeedSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_speed_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverSpeedSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x42, 0x1e, 0xef, 0x3f, 0x56, 0x7a, 0x6a, 0x89,
      0x9b, 0x5f, 0xbf, 0x4e, 0x87, 0xef, 0x48, 0xe5,
      0xdf, 0xb2, 0xe8, 0x2e, 0x1f, 0x23, 0x9c, 0xca,
      0x6e, 0xc0, 0xcd, 0xcc, 0x85, 0x9e, 0x19, 0x98,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverSpeedSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverSpeedSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__speed_body_x[] = "speed_body_x";
static char px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__speed_body_y[] = "speed_body_y";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverSpeedSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__speed_body_x, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedSetpoint__FIELD_NAME__speed_body_y, 12, 12},
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
px4_msgs__msg__RoverSpeedSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverSpeedSetpoint__TYPE_NAME, 31, 31},
      {px4_msgs__msg__RoverSpeedSetpoint__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Speed Setpoint\n"
  "\n"
  "uint64 timestamp      # [us] Time since system start\n"
  "float32 speed_body_x  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction\n"
  "float32 speed_body_y  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverSpeedSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverSpeedSetpoint__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 355, 355},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverSpeedSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverSpeedSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
