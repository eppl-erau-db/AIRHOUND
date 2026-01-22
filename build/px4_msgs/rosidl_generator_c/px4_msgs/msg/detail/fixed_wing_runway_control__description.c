// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FixedWingRunwayControl.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/fixed_wing_runway_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FixedWingRunwayControl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0x4e, 0x7f, 0x52, 0xb9, 0xb7, 0xf7, 0xc3,
      0x8a, 0xac, 0x4b, 0xdc, 0x69, 0xf9, 0x24, 0x37,
      0xe7, 0xde, 0x51, 0x77, 0x0d, 0xb8, 0xef, 0x90,
      0x01, 0x63, 0x8d, 0xe2, 0x14, 0x58, 0xfc, 0xb9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FixedWingRunwayControl__TYPE_NAME[] = "px4_msgs/msg/FixedWingRunwayControl";

// Define type names, field names, and default values
static char px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__wheel_steering_enabled[] = "wheel_steering_enabled";
static char px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__wheel_steering_nudging_rate[] = "wheel_steering_nudging_rate";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FixedWingRunwayControl__FIELDS[] = {
  {
    {px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__wheel_steering_enabled, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FixedWingRunwayControl__FIELD_NAME__wheel_steering_nudging_rate, 27, 27},
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
px4_msgs__msg__FixedWingRunwayControl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FixedWingRunwayControl__TYPE_NAME, 35, 35},
      {px4_msgs__msg__FixedWingRunwayControl__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Auxiliary control fields for fixed-wing runway takeoff/landing\n"
  "\n"
  "# Passes information from the FixedWingModeManager to the FixedWingAttitudeController\n"
  "\n"
  "uint64 timestamp # [us] time since system start\n"
  "\n"
  "bool wheel_steering_enabled\\t\\t# Flag that enables the wheel steering.\n"
  "float32 wheel_steering_nudging_rate\\t# [norm] [@range -1, 1] [FRD] Manual wheel nudging, added to controller output. NAN is interpreted as 0.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FixedWingRunwayControl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FixedWingRunwayControl__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 412, 412},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FixedWingRunwayControl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FixedWingRunwayControl__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
