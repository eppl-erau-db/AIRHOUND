// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TrajectorySetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/trajectory_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TrajectorySetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0x7d, 0xeb, 0x0c, 0xb7, 0xdb, 0x5e, 0x52,
      0xb4, 0x41, 0x16, 0x9a, 0x28, 0x4b, 0x90, 0x18,
      0x8c, 0x4e, 0xfd, 0xf0, 0x6b, 0x30, 0x2d, 0x55,
      0xcb, 0x1d, 0xb3, 0x3c, 0xa0, 0x35, 0x8c, 0x1a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TrajectorySetpoint__TYPE_NAME[] = "px4_msgs/msg/TrajectorySetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__velocity[] = "velocity";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__acceleration[] = "acceleration";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__jerk[] = "jerk";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__yawspeed[] = "yawspeed";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TrajectorySetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__jerk, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint__FIELD_NAME__yawspeed, 8, 8},
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
px4_msgs__msg__TrajectorySetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TrajectorySetpoint__TYPE_NAME, 31, 31},
      {px4_msgs__msg__TrajectorySetpoint__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Trajectory setpoint in NED frame\n"
  "# Input to PID position controller.\n"
  "# Needs to be kinematically consistent and feasible for smooth flight.\n"
  "# setting a value to NaN means the state should not be controlled\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "# NED local world frame\n"
  "float32[3] position # in meters\n"
  "float32[3] velocity # in meters/second\n"
  "float32[3] acceleration # in meters/second^2\n"
  "float32[3] jerk # in meters/second^3 (for logging only)\n"
  "\n"
  "float32 yaw # euler angle of desired attitude in radians -PI..+PI\n"
  "float32 yawspeed # angular velocity around NED frame z-axis in radians/second";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TrajectorySetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TrajectorySetpoint__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 637, 637},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TrajectorySetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TrajectorySetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
