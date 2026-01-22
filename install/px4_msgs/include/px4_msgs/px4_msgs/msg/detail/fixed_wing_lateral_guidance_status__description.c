// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FixedWingLateralGuidanceStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/fixed_wing_lateral_guidance_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FixedWingLateralGuidanceStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3d, 0x9d, 0x9b, 0xac, 0xa8, 0x16, 0x3c, 0x76,
      0xfd, 0xa9, 0xa4, 0xe8, 0xcb, 0x61, 0x4d, 0xac,
      0x38, 0xf7, 0x2d, 0x1b, 0xa5, 0x86, 0xe5, 0x4b,
      0x09, 0x62, 0x8a, 0x0b, 0x7e, 0xc9, 0xf1, 0x4a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FixedWingLateralGuidanceStatus__TYPE_NAME[] = "px4_msgs/msg/FixedWingLateralGuidanceStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__course_setpoint[] = "course_setpoint";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__lateral_acceleration_ff[] = "lateral_acceleration_ff";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__bearing_feas[] = "bearing_feas";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__bearing_feas_on_track[] = "bearing_feas_on_track";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__signed_track_error[] = "signed_track_error";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__track_error_bound[] = "track_error_bound";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__adapted_period[] = "adapted_period";
static char px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__wind_est_valid[] = "wind_est_valid";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELDS[] = {
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__course_setpoint, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__lateral_acceleration_ff, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__bearing_feas, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__bearing_feas_on_track, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__signed_track_error, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__track_error_bound, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__adapted_period, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELD_NAME__wind_est_valid, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__FixedWingLateralGuidanceStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FixedWingLateralGuidanceStatus__TYPE_NAME, 43, 43},
      {px4_msgs__msg__FixedWingLateralGuidanceStatus__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fixed Wing Lateral Guidance Status message\n"
  "# Published by fw_pos_control module to report the resultant lateral setpoints and NPFG debug outputs\n"
  "\n"
  "uint64 timestamp                # time since system start (microseconds)\n"
  "\n"
  "float32 course_setpoint         # [rad] [@range -pi, pi] Desired direction of travel over ground w.r.t (true) North. Set by guidance law\n"
  "float32 lateral_acceleration_ff # [m/s^2] [FRD] lateral acceleration demand only for maintaining curvature\n"
  "float32 bearing_feas            # [@range 0,1] bearing feasibility\n"
  "float32 bearing_feas_on_track   # [@range 0,1] on-track bearing feasibility\n"
  "float32 signed_track_error      # [m] signed track error\n"
  "float32 track_error_bound       # [m] track error bound\n"
  "float32 adapted_period          # [s] adapted period (if auto-tuning enabled)\n"
  "uint8 wind_est_valid            # [boolean] true = wind estimate is valid and/or being used by controller (also indicates if wind estimate usage is disabled despite being valid)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FixedWingLateralGuidanceStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FixedWingLateralGuidanceStatus__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 978, 978},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FixedWingLateralGuidanceStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FixedWingLateralGuidanceStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
