// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FollowTargetStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/follow_target_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FollowTargetStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaa, 0xda, 0x7f, 0x19, 0xfe, 0xb6, 0x4e, 0x0e,
      0x2d, 0x94, 0x52, 0x2e, 0x7c, 0xe3, 0xbc, 0xbb,
      0x2b, 0x64, 0x52, 0x99, 0x2b, 0x50, 0x9b, 0x5d,
      0x80, 0x12, 0x1f, 0xea, 0xf5, 0xfc, 0x49, 0xa6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FollowTargetStatus__TYPE_NAME[] = "px4_msgs/msg/FollowTargetStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__tracked_target_course[] = "tracked_target_course";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__follow_angle[] = "follow_angle";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__orbit_angle_setpoint[] = "orbit_angle_setpoint";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__angular_rate_setpoint[] = "angular_rate_setpoint";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__desired_position_raw[] = "desired_position_raw";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__in_emergency_ascent[] = "in_emergency_ascent";
static char px4_msgs__msg__FollowTargetStatus__FIELD_NAME__gimbal_pitch[] = "gimbal_pitch";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FollowTargetStatus__FIELDS[] = {
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__tracked_target_course, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__follow_angle, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__orbit_angle_setpoint, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__angular_rate_setpoint, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__desired_position_raw, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__in_emergency_ascent, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FollowTargetStatus__FIELD_NAME__gimbal_pitch, 12, 12},
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
px4_msgs__msg__FollowTargetStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FollowTargetStatus__TYPE_NAME, 31, 31},
      {px4_msgs__msg__FollowTargetStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                  # [microseconds] time since system start\n"
  "\n"
  "float32 tracked_target_course     # [rad] Tracked target course in NED local frame (North is course zero)\n"
  "float32 follow_angle              # [rad] Current follow angle setting\n"
  "\n"
  "float32 orbit_angle_setpoint      # [rad] Current orbit angle setpoint from the smooth trajectory generator\n"
  "float32 angular_rate_setpoint     # [rad/s] Angular rate commanded from Jerk-limited Orbit Angle trajectory for Orbit Angle\n"
  "\n"
  "float32[3] desired_position_raw   # [m] Raw 'idealistic' desired drone position if a drone could teleport from place to places\n"
  "\n"
  "bool in_emergency_ascent          # [bool] True when doing emergency ascent (when distance to ground is below safety altitude)\n"
  "float32 gimbal_pitch              # [rad] Gimbal pitch commanded to track target in the center of the frame";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FollowTargetStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FollowTargetStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 850, 850},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FollowTargetStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FollowTargetStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
