// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LateralControlConfiguration.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/lateral_control_configuration__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LateralControlConfiguration__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x44, 0x5a, 0xd3, 0x9a, 0x14, 0x05, 0x7a, 0x88,
      0x0b, 0x31, 0x2a, 0x2a, 0x47, 0xb4, 0xb7, 0xbe,
      0xd5, 0x0d, 0x27, 0xeb, 0x27, 0x12, 0x6e, 0x37,
      0x97, 0x51, 0xe3, 0xa2, 0x29, 0xf4, 0x8e, 0x30,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LateralControlConfiguration__TYPE_NAME[] = "px4_msgs/msg/LateralControlConfiguration";

// Define type names, field names, and default values
static char px4_msgs__msg__LateralControlConfiguration__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LateralControlConfiguration__FIELD_NAME__lateral_accel_max[] = "lateral_accel_max";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LateralControlConfiguration__FIELDS[] = {
  {
    {px4_msgs__msg__LateralControlConfiguration__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LateralControlConfiguration__FIELD_NAME__lateral_accel_max, 17, 17},
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
px4_msgs__msg__LateralControlConfiguration__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LateralControlConfiguration__TYPE_NAME, 40, 40},
      {px4_msgs__msg__LateralControlConfiguration__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fixed Wing Lateral Control Configuration message\n"
  "# Used by the fw_lateral_longitudinal_control module to constrain FixedWingLateralSetpoint messages.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp          # time since system start (microseconds)\n"
  "\n"
  "float32 lateral_accel_max # [m/s^2] currently maps to a maximum roll angle, accel_max = tan(roll_max) * GRAVITY";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LateralControlConfiguration__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LateralControlConfiguration__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 361, 361},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LateralControlConfiguration__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LateralControlConfiguration__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
