// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverPositionSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_position_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverPositionSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x9d, 0xc8, 0xee, 0xaf, 0x7f, 0x80, 0xc9,
      0xeb, 0x9c, 0x7f, 0xe1, 0x6c, 0x16, 0x2e, 0x4a,
      0x73, 0x79, 0x34, 0xf2, 0x20, 0x35, 0x7b, 0xd5,
      0xdf, 0x7d, 0x00, 0xef, 0xea, 0xad, 0xa5, 0xaf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverPositionSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__position_ned[] = "position_ned";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__start_ned[] = "start_ned";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__cruising_speed[] = "cruising_speed";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__arrival_speed[] = "arrival_speed";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverPositionSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__position_ned, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__start_ned, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__cruising_speed, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__arrival_speed, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__yaw, 3, 3},
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
px4_msgs__msg__RoverPositionSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__RoverPositionSetpoint__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Position Setpoint\n"
  "\n"
  "uint64 timestamp         # [us] Time since system start\n"
  "float32[2] position_ned  # [m] [@range -inf, inf] [@frame NED] Target position\n"
  "float32[2] start_ned     # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to vehicle position] Start position which specifies a line for the rover to track\n"
  "float32 cruising_speed   # [m/s] [@range 0, inf] [@invalid NaN Defaults to maximum speed] Cruising speed\n"
  "float32 arrival_speed    # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed the rover should arrive at the target with\n"
  "float32 yaw              # [rad] [@range -pi,pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Mecanum only: Specify vehicle yaw during travel";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverPositionSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 711, 711},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverPositionSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverPositionSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
