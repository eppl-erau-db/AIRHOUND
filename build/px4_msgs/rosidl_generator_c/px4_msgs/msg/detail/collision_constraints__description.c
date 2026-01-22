// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CollisionConstraints.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/collision_constraints__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CollisionConstraints__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9f, 0x57, 0x7d, 0x24, 0x03, 0x52, 0x1a, 0x50,
      0xad, 0x0d, 0xc4, 0xbd, 0xae, 0xb0, 0xe9, 0x07,
      0x0b, 0xd4, 0x40, 0x0b, 0x65, 0x2c, 0x3c, 0x32,
      0x7b, 0x93, 0xe3, 0x84, 0xa0, 0xce, 0x08, 0xb8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CollisionConstraints__TYPE_NAME[] = "px4_msgs/msg/CollisionConstraints";

// Define type names, field names, and default values
static char px4_msgs__msg__CollisionConstraints__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CollisionConstraints__FIELD_NAME__original_setpoint[] = "original_setpoint";
static char px4_msgs__msg__CollisionConstraints__FIELD_NAME__adapted_setpoint[] = "adapted_setpoint";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CollisionConstraints__FIELDS[] = {
  {
    {px4_msgs__msg__CollisionConstraints__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionConstraints__FIELD_NAME__original_setpoint, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionConstraints__FIELD_NAME__adapted_setpoint, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__CollisionConstraints__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CollisionConstraints__TYPE_NAME, 33, 33},
      {px4_msgs__msg__CollisionConstraints__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Local setpoint constraints in NED frame\n"
  "# setting something to NaN means that no limit is provided\n"
  "\n"
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "\n"
  "float32[2] original_setpoint   # velocities demanded\n"
  "float32[2] adapted_setpoint    # velocities allowed";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CollisionConstraints__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CollisionConstraints__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 266, 266},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CollisionConstraints__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CollisionConstraints__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
