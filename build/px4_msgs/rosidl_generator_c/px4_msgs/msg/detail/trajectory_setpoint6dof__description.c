// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TrajectorySetpoint6dof.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/trajectory_setpoint6dof__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TrajectorySetpoint6dof__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xce, 0xb2, 0xe4, 0xd0, 0xa1, 0xf4, 0x0b, 0xe0,
      0x7b, 0x4b, 0x17, 0xd2, 0xc6, 0x1b, 0xd8, 0x3a,
      0x9c, 0xe5, 0x22, 0xc2, 0xe6, 0x7e, 0xad, 0x02,
      0xdf, 0x0c, 0xd2, 0x93, 0x2a, 0x25, 0xf0, 0xa7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TrajectorySetpoint6dof__TYPE_NAME[] = "px4_msgs/msg/TrajectorySetpoint6dof";

// Define type names, field names, and default values
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__velocity[] = "velocity";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__acceleration[] = "acceleration";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__jerk[] = "jerk";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__quaternion[] = "quaternion";
static char px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__angular_velocity[] = "angular_velocity";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TrajectorySetpoint6dof__FIELDS[] = {
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__jerk, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__quaternion, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectorySetpoint6dof__FIELD_NAME__angular_velocity, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TrajectorySetpoint6dof__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TrajectorySetpoint6dof__TYPE_NAME, 35, 35},
      {px4_msgs__msg__TrajectorySetpoint6dof__FIELDS, 7, 7},
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
  "# Input to position controller.\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "# NED local world frame\n"
  "float32[3] position               # in meters\n"
  "float32[3] velocity               # in meters/second\n"
  "float32[3] acceleration           # in meters/second^2\n"
  "float32[3] jerk                   # in meters/second^3 (for logging only)\n"
  "\n"
  "float32[4] quaternion             # unit quaternion\n"
  "float32[3] angular_velocity       # angular velocity in radians/second";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TrajectorySetpoint6dof__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TrajectorySetpoint6dof__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 503, 503},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TrajectorySetpoint6dof__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TrajectorySetpoint6dof__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
