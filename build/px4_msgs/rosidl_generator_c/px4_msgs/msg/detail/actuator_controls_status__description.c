// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActuatorControlsStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/actuator_controls_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActuatorControlsStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x31, 0x19, 0x2b, 0x45, 0xbb, 0xc4, 0x2c, 0x5b,
      0xe5, 0x6e, 0xf2, 0xd4, 0xfd, 0x56, 0xd6, 0x1a,
      0x48, 0x5b, 0x40, 0x44, 0x2f, 0xe7, 0x8b, 0xdb,
      0xd8, 0x12, 0x0a, 0xc0, 0x67, 0xc3, 0x26, 0x20,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActuatorControlsStatus__TYPE_NAME[] = "px4_msgs/msg/ActuatorControlsStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__ActuatorControlsStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActuatorControlsStatus__FIELD_NAME__control_power[] = "control_power";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActuatorControlsStatus__FIELDS[] = {
  {
    {px4_msgs__msg__ActuatorControlsStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorControlsStatus__FIELD_NAME__control_power, 13, 13},
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
px4_msgs__msg__ActuatorControlsStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActuatorControlsStatus__TYPE_NAME, 35, 35},
      {px4_msgs__msg__ActuatorControlsStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32[3] control_power\n"
  "\n"
  "# TOPICS actuator_controls_status_0 actuator_controls_status_1";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActuatorControlsStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActuatorControlsStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 150, 150},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActuatorControlsStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActuatorControlsStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
