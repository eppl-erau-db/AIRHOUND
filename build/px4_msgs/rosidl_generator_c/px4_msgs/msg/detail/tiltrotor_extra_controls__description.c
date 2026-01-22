// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TiltrotorExtraControls.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/tiltrotor_extra_controls__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TiltrotorExtraControls__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0a, 0x41, 0xa9, 0x00, 0x35, 0xba, 0x52, 0xa5,
      0x3f, 0x24, 0x5c, 0xbb, 0xf5, 0xf5, 0x3c, 0x7b,
      0x98, 0xdd, 0x6d, 0xd1, 0xbd, 0xc0, 0x2b, 0xba,
      0xc9, 0x02, 0x8a, 0xe6, 0x52, 0x17, 0x9b, 0x83,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TiltrotorExtraControls__TYPE_NAME[] = "px4_msgs/msg/TiltrotorExtraControls";

// Define type names, field names, and default values
static char px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__collective_tilt_normalized_setpoint[] = "collective_tilt_normalized_setpoint";
static char px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__collective_thrust_normalized_setpoint[] = "collective_thrust_normalized_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TiltrotorExtraControls__FIELDS[] = {
  {
    {px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__collective_tilt_normalized_setpoint, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TiltrotorExtraControls__FIELD_NAME__collective_thrust_normalized_setpoint, 37, 37},
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
px4_msgs__msg__TiltrotorExtraControls__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TiltrotorExtraControls__TYPE_NAME, 35, 35},
      {px4_msgs__msg__TiltrotorExtraControls__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 collective_tilt_normalized_setpoint\\t# Collective tilt angle of motors of tiltrotor, 0: vertical, 1: horizontal [0, 1]\n"
  "float32 collective_thrust_normalized_setpoint \\t# Collective thrust setpoint [0, 1]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TiltrotorExtraControls__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TiltrotorExtraControls__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 268, 268},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TiltrotorExtraControls__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TiltrotorExtraControls__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
