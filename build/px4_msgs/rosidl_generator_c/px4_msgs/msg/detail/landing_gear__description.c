// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LandingGear.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/landing_gear__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LandingGear__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdf, 0x35, 0xd4, 0xa5, 0x9c, 0x22, 0x49, 0x33,
      0xe8, 0x74, 0xec, 0x67, 0x3e, 0xa4, 0x5d, 0x9e,
      0xf9, 0xa4, 0x7e, 0x21, 0xac, 0x0b, 0x55, 0xe1,
      0x39, 0x02, 0xad, 0x9e, 0xaf, 0x67, 0x8a, 0x3c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LandingGear__TYPE_NAME[] = "px4_msgs/msg/LandingGear";

// Define type names, field names, and default values
static char px4_msgs__msg__LandingGear__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LandingGear__FIELD_NAME__landing_gear[] = "landing_gear";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LandingGear__FIELDS[] = {
  {
    {px4_msgs__msg__LandingGear__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingGear__FIELD_NAME__landing_gear, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__LandingGear__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LandingGear__TYPE_NAME, 24, 24},
      {px4_msgs__msg__LandingGear__FIELDS, 2, 2},
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
  "int8 GEAR_UP = 1 # landing gear up\n"
  "int8 GEAR_DOWN = -1 # landing gear down\n"
  "int8 GEAR_KEEP = 0 # keep the current state\n"
  "\n"
  "int8 landing_gear";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LandingGear__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LandingGear__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 197, 197},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LandingGear__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LandingGear__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
