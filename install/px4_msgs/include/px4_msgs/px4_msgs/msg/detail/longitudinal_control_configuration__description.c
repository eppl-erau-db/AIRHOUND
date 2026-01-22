// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LongitudinalControlConfiguration.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/longitudinal_control_configuration__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LongitudinalControlConfiguration__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x65, 0x17, 0x12, 0xb9, 0xf4, 0xfb, 0xde, 0x1d,
      0x49, 0xfa, 0x64, 0xc2, 0x0f, 0x96, 0x39, 0x5b,
      0x01, 0xbc, 0x95, 0xdf, 0xc1, 0xab, 0xfb, 0x6a,
      0xb9, 0x3c, 0xb7, 0x19, 0x38, 0x99, 0x2b, 0x57,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LongitudinalControlConfiguration__TYPE_NAME[] = "px4_msgs/msg/LongitudinalControlConfiguration";

// Define type names, field names, and default values
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__pitch_min[] = "pitch_min";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__pitch_max[] = "pitch_max";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__throttle_min[] = "throttle_min";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__throttle_max[] = "throttle_max";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__climb_rate_target[] = "climb_rate_target";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__sink_rate_target[] = "sink_rate_target";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__speed_weight[] = "speed_weight";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__enforce_low_height_condition[] = "enforce_low_height_condition";
static char px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__disable_underspeed_protection[] = "disable_underspeed_protection";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LongitudinalControlConfiguration__FIELDS[] = {
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__pitch_min, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__pitch_max, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__throttle_min, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__throttle_max, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__climb_rate_target, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__sink_rate_target, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__speed_weight, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__enforce_low_height_condition, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LongitudinalControlConfiguration__FIELD_NAME__disable_underspeed_protection, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__LongitudinalControlConfiguration__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LongitudinalControlConfiguration__TYPE_NAME, 45, 45},
      {px4_msgs__msg__LongitudinalControlConfiguration__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fixed Wing Longitudinal Control Configuration message\n"
  "# Used by the fw_lateral_longitudinal_control module and TECS to constrain FixedWingLongitudinalSetpoint messages\n"
  "# and configure the resultant setpoints.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "\n"
  "float32 pitch_min   \\t\\t\\t# [rad][@range -pi, pi] defaults to FW_P_LIM_MIN if NAN.\n"
  "float32 pitch_max   \\t\\t\\t# [rad][@range -pi, pi] defaults to FW_P_LIM_MAX if NAN.\n"
  "float32 throttle_min \\t\\t\\t# [norm] [@range 0,1] deaults to FW_THR_MIN if NAN.\n"
  "float32 throttle_max \\t\\t\\t# [norm] [@range 0,1] defaults to FW_THR_MAX if NAN.\n"
  "float32 climb_rate_target \\t\\t# [m/s] target climbrate to change altitude. Defaults to FW_T_CLIMB_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.\n"
  "float32 sink_rate_target \\t\\t# [m/s] target sinkrate to change altitude. Defaults to FW_T_SINK_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.\n"
  "float32 speed_weight \\t\\t\\t# [@range 0,2], 0=pitch controls altitude only, 2=pitch controls airspeed only\n"
  "bool enforce_low_height_condition \\t# [boolean] if true, the altitude controller is configured with an alternative timeconstant for tighter altitude tracking\n"
  "bool disable_underspeed_protection \\t# [boolean] if true, underspeed handling is disabled in the altitude controller";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LongitudinalControlConfiguration__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LongitudinalControlConfiguration__TYPE_NAME, 45, 45},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1372, 1372},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LongitudinalControlConfiguration__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LongitudinalControlConfiguration__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
