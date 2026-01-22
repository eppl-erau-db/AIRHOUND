// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FixedWingLateralSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/fixed_wing_lateral_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FixedWingLateralSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0x98, 0x99, 0xbb, 0x43, 0x9d, 0xe2, 0x78,
      0x53, 0x1c, 0x37, 0x27, 0x6e, 0x21, 0x3b, 0xb9,
      0xf0, 0x81, 0x3e, 0xe9, 0xf3, 0x9a, 0x4f, 0xba,
      0x78, 0xd2, 0xd0, 0x2c, 0xa8, 0xb1, 0x7c, 0xe5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FixedWingLateralSetpoint__TYPE_NAME[] = "px4_msgs/msg/FixedWingLateralSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__course[] = "course";
static char px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__airspeed_direction[] = "airspeed_direction";
static char px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__lateral_acceleration[] = "lateral_acceleration";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FixedWingLateralSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__course, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__airspeed_direction, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralSetpoint__FIELD_NAME__lateral_acceleration, 20, 20},
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
px4_msgs__msg__FixedWingLateralSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FixedWingLateralSetpoint__TYPE_NAME, 37, 37},
      {px4_msgs__msg__FixedWingLateralSetpoint__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fixed Wing Lateral Setpoint message\n"
  "# Used by the fw_lateral_longitudinal_control module\n"
  "# At least one of course, airspeed_direction, or lateral_acceleration must be finite.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "\n"
  "float32 course \\t\\t\\t\\t# [rad] [@range -pi, pi] Desired direction of travel over ground w.r.t (true) North. NAN if not controlled directly.\n"
  "float32 airspeed_direction    \\t\\t# [rad] [@range -pi, pi] Desired horizontal angle of airspeed vector w.r.t. (true) North. Same as vehicle heading if in the absence of sideslip. NAN if not controlled directly, takes precedence over course if finite.\n"
  "float32 lateral_acceleration \\t\\t# [m/s^2] [FRD] Lateral acceleration setpoint. NAN if not controlled directly, used as feedforward if either course setpoint or airspeed_direction is finite.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FixedWingLateralSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FixedWingLateralSetpoint__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 862, 862},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FixedWingLateralSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FixedWingLateralSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
