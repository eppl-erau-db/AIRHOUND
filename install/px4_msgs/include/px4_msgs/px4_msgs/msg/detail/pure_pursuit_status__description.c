// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PurePursuitStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/pure_pursuit_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PurePursuitStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4f, 0x29, 0xad, 0x14, 0xc0, 0xf3, 0xd3, 0xc2,
      0xdc, 0x3f, 0xe4, 0x83, 0xe0, 0xb6, 0x00, 0x1b,
      0x27, 0x41, 0xd1, 0x9a, 0x1e, 0xfb, 0x00, 0x80,
      0xb5, 0x31, 0x14, 0xc8, 0xe2, 0x9a, 0xac, 0x28,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__PurePursuitStatus__TYPE_NAME[] = "px4_msgs/msg/PurePursuitStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__lookahead_distance[] = "lookahead_distance";
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__target_bearing[] = "target_bearing";
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__crosstrack_error[] = "crosstrack_error";
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__distance_to_waypoint[] = "distance_to_waypoint";
static char px4_msgs__msg__PurePursuitStatus__FIELD_NAME__bearing_to_waypoint[] = "bearing_to_waypoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PurePursuitStatus__FIELDS[] = {
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__lookahead_distance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__target_bearing, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__crosstrack_error, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__distance_to_waypoint, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PurePursuitStatus__FIELD_NAME__bearing_to_waypoint, 19, 19},
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
px4_msgs__msg__PurePursuitStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PurePursuitStatus__TYPE_NAME, 30, 30},
      {px4_msgs__msg__PurePursuitStatus__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Pure pursuit status\n"
  "\n"
  "uint64 timestamp              # [us] Time since system start\n"
  "float32 lookahead_distance    # [m] [@range 0, inf] Lookahead distance of pure the pursuit controller\n"
  "float32 target_bearing        # [rad] [@range -pi, pi] [@frame NED] Target bearing calculated by the pure pursuit controller\n"
  "float32 crosstrack_error      # [m] [@range -inf (Left of the path), inf (Right of the path)] Shortest distance from the vehicle to the path\n"
  "float32 distance_to_waypoint  # [m] [@range -inf, inf]Distance from the vehicle to the current waypoint\n"
  "float32 bearing_to_waypoint   # [rad] [@range -pi, pi] [@frame NED]Bearing towards current waypoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PurePursuitStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PurePursuitStatus__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 656, 656},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PurePursuitStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PurePursuitStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
