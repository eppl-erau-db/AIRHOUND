// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FixedWingLateralStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/fixed_wing_lateral_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FixedWingLateralStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0x3f, 0xd5, 0xc3, 0x34, 0x86, 0xb4, 0x69,
      0xe0, 0xb8, 0x44, 0x1d, 0x44, 0x7b, 0xfb, 0x7c,
      0xd4, 0x2d, 0xbd, 0xd1, 0x08, 0xb4, 0xf4, 0xb0,
      0x2f, 0xa7, 0x64, 0x52, 0x19, 0x06, 0x06, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FixedWingLateralStatus__TYPE_NAME[] = "px4_msgs/msg/FixedWingLateralStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__lateral_acceleration_setpoint[] = "lateral_acceleration_setpoint";
static char px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__can_run_factor[] = "can_run_factor";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FixedWingLateralStatus__FIELDS[] = {
  {
    {px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__lateral_acceleration_setpoint, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingLateralStatus__FIELD_NAME__can_run_factor, 14, 14},
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
px4_msgs__msg__FixedWingLateralStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FixedWingLateralStatus__TYPE_NAME, 35, 35},
      {px4_msgs__msg__FixedWingLateralStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fixed Wing Lateral Status message\n"
  "# Published by the fw_lateral_longitudinal_control module to report the resultant lateral setpoint\n"
  "\n"
  "uint64 timestamp                         # time since system start (microseconds)\n"
  "\n"
  "float32 lateral_acceleration_setpoint    # [m/s^2] [FRD] resultant lateral acceleration setpoint\n"
  "float32 can_run_factor \\t \\t         # [norm] [@range 0, 1] estimate of certainty of the correct functionality of the npfg roll setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FixedWingLateralStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FixedWingLateralStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 451, 451},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FixedWingLateralStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FixedWingLateralStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
